# SPDX-License-Identifier: GPL-3.0-or-later
"""Armature construction from a RigPlan, in three strict phases.

Phase 1 (Object mode) creates datablocks, Phase 2 is ONE Edit-mode session
that creates every bone, Phase 3 (Pose mode) configures channels and
constraints. The phases never interleave: an EditBone reference is dead the
moment Edit mode ends, and touching one crashes Blender rather than raising.

Bone convention: local +Y is the joint DOF axis, +Z comes from the
manifest's secondary_axis so roll is deterministic across exports.
"""

import os
from dataclasses import dataclass, field
from typing import Dict, List

try:
    import bpy
    from mathutils import Matrix, Vector
except ImportError:
    bpy = None
    Matrix = None
    Vector = None

from . import constraints, drivers, loops
from .graph import RigPlan, swing_cone

_RIG_NAME_FALLBACK = "SW_Rig"
_HELPERS_COLLECTION = "SW_helpers"


def _rig_name(manifest) -> str:
    """One rig per assembly: the STEP file's base name keys the collection
    and armature, so building a second assembly's rig sits beside the first
    instead of replacing it."""
    try:
        base = os.path.splitext(os.path.basename(manifest.step_file or ""))[0]
    except (AttributeError, TypeError):
        base = ""
    return base + "_Rig" if base else _RIG_NAME_FALLBACK
_HELPER_LENGTH_M = 0.02
_DEFAULT_BBOX_DIAG_M = 0.4
# Width of a path joint's shrinkwrap ribbon: the bone can land up to half of
# it off the true path, so it buys accuracy at the cost of nothing but
# visibility, which _make_path_rail restores by drawing the ribbon as wire.
_RAIL_WIDTH = 2e-5
# How far a path joint's rest point may sit off its own sampled curve and
# still be treated as sampling error rather than bad data.
_RAIL_SNAP = 1e-3


@dataclass
class BuildResult:
    armature_object: object = None
    collection: object = None
    bone_names: Dict[str, str] = field(default_factory=dict)     # group id -> bone name
    helper_names: Dict[str, str] = field(default_factory=dict)   # loop id -> bone name
    effector_names: Dict[str, str] = field(default_factory=dict)  # loop id -> bone name
    tangent_helper_names: Dict[str, str] = field(default_factory=dict)  # group id -> bone name
    contact_mesh_names: Dict[str, str] = field(default_factory=dict)  # joint id -> rail or patch object name
    # Swing-cone balls AND cone_spin collapses: bone_names[gid] is the
    # hidden DEF bone (geometry and child bones ride the clamped result);
    # the visible user handle is here.
    ball_ctrl_names: Dict[str, str] = field(default_factory=dict)   # group id -> bone name
    ball_pole_names: Dict[str, str] = field(default_factory=dict)
    ball_goal_names: Dict[str, str] = field(default_factory=dict)
    cone_frame_names: Dict[str, str] = field(default_factory=dict)  # cone_spin plane frame
    warnings: List[str] = field(default_factory=list)


def _unit_scale(context):
    try:
        scale = float(context.scene.unit_settings.scale_length)
    except (AttributeError, TypeError):
        return 1.0
    # Blender units per metre; the manifest is metres everywhere.
    return 1.0 / scale if scale > 0.0 else 1.0


def _normalized(v):
    vec = Vector(v)
    if vec.length < 1e-9:
        return None
    return vec.normalized()


def _frame_matrix(y_axis, secondary, translation):
    """Columns x, y, z with y the DOF axis; z from the secondary axis
    orthogonalised against y, falling back to global Z then X when the
    secondary is missing or parallel."""
    y = _normalized(y_axis)
    if y is None:
        y = Vector((0.0, 1.0, 0.0))
    z = None
    for candidate in (secondary, (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)):
        if candidate is None:
            continue
        c = Vector(candidate)
        ortho = c - y * c.dot(y)
        if ortho.length > 1e-6:
            z = ortho.normalized()
            break
    x = y.cross(z)
    t = Vector(translation)
    return Matrix((
        (x[0], y[0], z[0], t[0]),
        (x[1], y[1], z[1], t[1]),
        (x[2], y[2], z[2], t[2]),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _mat4_to_matrix(rows, unit_scale):
    m = Matrix((tuple(rows[0]), tuple(rows[1]), tuple(rows[2]), tuple(rows[3])))
    for i in range(3):
        m[i][3] *= unit_scale
    return m


def _group_fallback_translation(manifest, group):
    comp_by_id = manifest.component_by_id()
    for cid in group.components:
        comp = comp_by_id.get(cid)
        if comp is not None:
            t = comp.transform
            return (t[0][3], t[1][3], t[2][3])
    return (0.0, 0.0, 0.0)


def _group_world_matrix(manifest, group, unit_scale):
    if group.frame is not None:
        return _mat4_to_matrix(group.frame, unit_scale)
    t = _group_fallback_translation(manifest, group)
    m = Matrix.Identity(4)
    for i in range(3):
        m[i][3] = t[i] * unit_scale
    return m


def _bone_rest_matrix(manifest, bone_plan, unit_scale):
    if bone_plan.root:
        # The root rests AT the assembly origin, world-aligned. A grounded
        # group's manifest frame is whichever member the exporter anchored
        # it to (live 2026-08-23: the flexible-sub "baseplate" bone landed
        # on the hinge boss) — for static geometry the origin is the one
        # placement that never surprises.
        return Matrix.Identity(4)
    joint = bone_plan.joint
    collapsed = bone_plan.collapsed
    if collapsed is not None:
        # Contact-aligned frame so the channel locks mean the contact:
        #   planar_spin: Y = spin axis, Z = the contact plane normal
        #     (lock loc Z = stay on the plane; rot X locked = no tilt).
        #   planar_ball / slide_ball: Y = the carrier's axis (plane normal /
        #     slide direction) — same convention as planar and prismatic.
        #   orbit_spin: Y = spin axis, secondary as exported.
        j1 = collapsed.carrier_joint
        j2 = collapsed.spin_joint
        origin = j2.origin if j2.origin is not None else \
            _group_fallback_translation(manifest, bone_plan.group)
        if collapsed.kind in ("planar_spin", "cone_spin"):
            m = _frame_matrix(j2.axis, j1.axis, origin)
        elif collapsed.kind == "orbit_spin":
            m = _frame_matrix(j2.axis, j2.secondary_axis, origin)
        else:  # planar_ball, slide_ball
            m = _frame_matrix(j1.axis, j1.secondary_axis, origin)
        for i in range(3):
            m[i][3] *= unit_scale
        return m
    if bone_plan.mirror_normal is not None:
        # Half of a mirror pair: BOTH bones rest with the SAME plane-aligned
        # orientation (local +Y = the mirror normal) at their own mirrored
        # positions — equal rest axes make the reflection exact per channel
        # (loc y negates, euler x/z negate), which is all the six sign-flip
        # drivers assume.
        origin = _group_fallback_translation(manifest, bone_plan.group)
        m = _frame_matrix(bone_plan.mirror_normal, None, origin)
        for i in range(3):
            m[i][3] *= unit_scale
        return m
    if swing_cone(joint):
        # The bone points along the child-side measured direction (the stud
        # axis), NOT along joint.axis — for a ball the axis is the
        # parent-fixed CONE axis the limit band is measured from. Roll from
        # the cone axis, so the frame is deterministic; at a rest inside the
        # cone the two may be parallel and _frame_matrix falls back.
        origin = joint.origin if joint.origin is not None else \
            _group_fallback_translation(manifest, bone_plan.group)
        m = _frame_matrix(joint.secondary_axis, joint.axis, origin)
        for i in range(3):
            m[i][3] *= unit_scale
        return m
    if joint is not None and joint.axis is not None:
        origin = joint.origin if joint.origin is not None else \
            _group_fallback_translation(manifest, bone_plan.group)
        m = _frame_matrix(joint.axis, joint.secondary_axis, origin)
        for i in range(3):
            m[i][3] *= unit_scale
        return m
    if joint is not None and joint.origin is not None:
        # A ball has no axis, but a world-aligned bone LIES about the pose:
        # the stud tilted 45° in SolidWorks still grew a vertical bone (live
        # corpus 04, 2026-08-23). The child part's own rotation is rigid
        # with the geometry, so the bone tilts exactly as the part does —
        # whatever local axis the part was modelled along.
        m = _group_component_rotation(manifest, bone_plan.group)
        for i in range(3):
            m[i][3] = joint.origin[i] * unit_scale
        return m
    return _group_world_matrix(manifest, bone_plan.group, unit_scale)


def _group_component_rotation(manifest, group):
    """The rotation of the group's first component, orthonormalized, as a
    4x4 with zero translation. Identity when the group owns no components
    (a carrier) or the transform is degenerate."""
    comp_by_id = manifest.component_by_id()
    for cid in group.components:
        comp = comp_by_id.get(cid)
        if comp is None:
            continue
        t = comp.transform
        x = Vector((t[0][0], t[1][0], t[2][0]))
        y = Vector((t[0][1], t[1][1], t[2][1]))
        if x.length < 1e-9 or y.length < 1e-9:
            break
        x.normalize()
        z = x.cross(y)
        if z.length < 1e-9:
            break
        z.normalize()
        y = z.cross(x)
        m = Matrix.Identity(4)
        for i in range(3):
            m[i][0], m[i][1], m[i][2] = x[i], y[i], z[i]
        return m
    return Matrix.Identity(4)


def _bone_length(group, unit_scale):
    diag = group.bbox_diag if group.bbox_diag else _DEFAULT_BBOX_DIAG_M
    return max(0.01, min(1.0, diag / 4.0)) * unit_scale


def _ensure_object_mode(context):
    """Object mode is PER-OBJECT since 2.8: context.mode reports only the
    active object, so a background armature can still sit in Pose mode —
    and deleting it (the rig rebuild does) freezes Blender (live
    2026-08-23, send-while-posing). Sweep every object's own mode."""
    view_layer = context.view_layer
    prev = view_layer.objects.active
    for obj in list(view_layer.objects):
        try:
            if obj.mode == "OBJECT":
                continue
            view_layer.objects.active = obj
            bpy.ops.object.mode_set(mode="OBJECT")
        except (RuntimeError, ReferenceError):
            continue
    try:
        if prev is not None:
            view_layer.objects.active = prev
    except (RuntimeError, ReferenceError):
        pass


def _remove_previous_rig(collection):
    """Re-runs replace the rig instead of stacking name.001s. Geometry
    parented into the old rig is released with its world transform kept, so
    a rebuild never scatters the scene."""
    doomed = [o for o in list(collection.objects) if o.get("RIG_rig")]
    doomed_set = set(doomed)
    if not doomed:
        return
    for obj in bpy.data.objects:
        if obj.parent in doomed_set and obj not in doomed_set:
            world = obj.matrix_world.copy()
            obj.parent = None
            obj.matrix_world = world
    for obj in doomed:
        data = obj.data
        bpy.data.objects.remove(obj)
        if data is not None and data.users == 0:
            if isinstance(data, bpy.types.Armature):
                bpy.data.armatures.remove(data)
            elif isinstance(data, bpy.types.Curve):
                bpy.data.curves.remove(data)


def _thread_through(pts, rest, tolerance):
    """Inserts the joint's rest point into a sampled polyline so the path
    passes exactly through it.

    The manifest asserts two things that must agree — the curve's samples
    and where the mate holds the part — and a chord-sampled curve misses
    its own vertex by the sagitta. Since the rest pose is what the user
    sees on load, it wins, but only within `tolerance`: a rest point
    genuinely off the path is a data problem the relink warning should
    still report, not something to bend the geometry around.

    _pin_through does the same for a triangulated face."""
    best = (tolerance, -1, 0.0)
    for i in range(len(pts) - 1):
        span = pts[i + 1] - pts[i]
        if span.length_squared < 1e-18:
            continue
        t = (rest - pts[i]).dot(span) / span.length_squared
        if not 1e-6 < t < 1.0 - 1e-6:
            continue           # at a vertex: inserting would spur the rail
        d = (rest - (pts[i] + span * t)).length
        if d < best[0]:
            best = (d, i, t)
    if best[1] >= 0 and best[0] > 1e-9:
        pts.insert(best[1] + 1, rest)


def _make_path_rail(collection, joint, frame, unit_scale):
    """The surface a path joint's bone rides: a hairline RIBBON through the
    manifest's sampled points, placed through the same scene frame as the
    bones.

    Blender has no nearest-point-on-curve constraint, and Clamp To is not
    one: it maps the bone's coordinate along a single axis linearly into
    curve parameter, so a bone resting exactly ON the curve is teleported
    somewhere else along it and drags its geometry with it (live corpus 17,
    2026-08-23: 26 mm along an 0.88 m spline). Shrinkwrap NEAREST_SURFACE
    IS nearest-point, which makes the rest pose a fixed point of the
    constraint while a dragged bone still slides along the path.

    Two constraints on the shape follow: the ribbon must have FACES (the
    shrinkwrap BVH ignores loose edges and the constraint then silently
    does nothing), and the rails must stay a hair apart so every point of
    the surface is within half a width of the true path.

    Tagged RIG_rig (replaced on rebuild) and RIG_helper (invisible to
    matching and geometry parenting)."""
    pts = [frame @ Vector((p[0] * unit_scale, p[1] * unit_scale,
                           p[2] * unit_scale))
           for p in joint.path_points]
    if joint.origin is not None:
        _thread_through(pts, frame @ Vector((joint.origin[0] * unit_scale,
                                             joint.origin[1] * unit_scale,
                                             joint.origin[2] * unit_scale)),
                        _RAIL_SNAP * unit_scale)
    half = _RAIL_WIDTH * unit_scale * 0.5
    verts = []
    offset = None
    for i, p in enumerate(pts):
        tangent = pts[i + 1] - p if i < len(pts) - 1 else p - pts[i - 1]
        if tangent.length < 1e-12:
            tangent = Vector((0.0, 0.0, 1.0))
        tangent.normalize()
        if offset is None:
            offset = tangent.orthogonal().normalized()
        else:
            # Parallel transport rather than a fresh perpendicular per point:
            # an abrupt flip would fold the quad into a bow tie whose BVH
            # entry straddles the path.
            offset = offset - tangent * offset.dot(tangent)
            offset = (offset.normalized() if offset.length > 1e-9
                      else tangent.orthogonal().normalized())
        verts.append(tuple(p - offset * half))
        verts.append(tuple(p + offset * half))
    faces = [(2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2)
             for i in range(len(pts) - 1)]
    if joint.path_closed:
        faces.append((2 * len(pts) - 2, 2 * len(pts) - 1, 1, 0))

    data = bpy.data.meshes.new("SWTB_path_" + joint.id)
    data.from_pydata(verts, [], faces)
    data.update()
    obj = bpy.data.objects.new(data.name, data)
    obj["RIG_rig"] = True
    obj["RIG_helper"] = joint.id
    obj.hide_render = True
    # A ribbon this thin has no readable solid shading; drawn as wire it
    # reads as the path line it stands for.
    obj.display_type = "WIRE"
    collection.objects.link(obj)
    return obj


def _pin_through(verts, faces, rest, tolerance):
    """The 2-D twin of _thread_through: fans the triangle under the joint's
    rest point into three through the point itself, so a tessellated face
    passes exactly through the contact its mate defines. Same reasoning and
    the same tolerance gate — a chordal face misses its own mate vertex by
    the sagitta, and the rest pose is what the user sees on load."""
    best = (tolerance, -1)
    for i, tri in enumerate(faces):
        a, b, c = (verts[k] for k in tri)
        normal = (b - a).cross(c - a)
        if normal.length_squared < 1e-24:
            continue
        normal = normal.normalized()
        planar = rest - normal * (rest - a).dot(normal)
        # Inside test by the sign of each edge's cross product against the
        # face normal — a point outside the triangle belongs to a neighbour.
        inside = True
        for p, q in ((a, b), (b, c), (c, a)):
            if (q - p).cross(planar - p).dot(normal) < -1e-12:
                inside = False
                break
        if not inside:
            continue
        d = (rest - planar).length
        if d < best[0]:
            best = (d, i)
    if best[1] < 0 or best[0] <= 1e-9:
        return
    a, b, c = faces.pop(best[1])
    at = len(verts)
    verts.append(rest)
    faces.extend([(a, b, at), (b, c, at), (c, a, at)])


def _make_surface_patch(collection, joint, frame, unit_scale):
    """The face a surface joint's bone rides: the manifest's triangulation,
    placed through the same scene frame as the bones.

    Same machinery as _make_path_rail one dimension up — a shrinkwrap onto
    a real mesh — because SolidWorks will mate a point to a torus, a fillet
    or a loft, and none of those has a joint type to be. The patch is the
    face alone, not the whole body, so the point cannot wander onto a
    neighbouring face the mate never mentioned."""
    verts = [frame @ Vector((p[0] * unit_scale, p[1] * unit_scale,
                             p[2] * unit_scale))
             for p in joint.surface_points]
    faces = [tuple(t) for t in joint.surface_triangles]
    if joint.origin is not None:
        _pin_through(verts, faces,
                     frame @ Vector((joint.origin[0] * unit_scale,
                                     joint.origin[1] * unit_scale,
                                     joint.origin[2] * unit_scale)),
                     _RAIL_SNAP * unit_scale)

    data = bpy.data.meshes.new("SWTB_surface_" + joint.id)
    data.from_pydata([tuple(v) for v in verts], [], faces)
    data.update()
    obj = bpy.data.objects.new(data.name, data)
    obj["RIG_rig"] = True
    obj["RIG_helper"] = joint.id
    obj.hide_render = True
    # Wire, like the path rail: the patch is a duplicate of a face that is
    # already in the scene, and a solid copy would z-fight with it.
    obj.display_type = "WIRE"
    collection.objects.link(obj)
    return obj


def build(context, manifest, plan: RigPlan, frame_rows=None) -> BuildResult:
    """Builds the armature, constraints, drivers and loop closures.
    plan already passed the dependency pre-flight in graph.build — nothing
    here is allowed to create a depsgraph cycle. Geometry is not required;
    the rig builds identically on an empty scene.

    frame_rows is the scene-frame transform from matching (manifest
    coordinates in Blender units -> where the geometry actually sits, e.g.
    the rotation a Y-up STEP import applied, cursor offset included). Every
    bone is placed through it, so the rig lands on the geometry whatever up
    axis the import used. Without a frame (no match run, or nothing to
    anchor one) the rig lands at the 3D cursor, like the STEP import itself
    does — never silently at the world origin. Limits and drivers are
    bone-local and need no adjustment."""
    result = BuildResult()
    result.warnings.extend(plan.warnings)
    unit_scale = _unit_scale(context)
    if frame_rows:
        frame = Matrix([tuple(r) for r in frame_rows])
    else:
        try:
            frame = Matrix.Translation(context.scene.cursor.location)
        except (AttributeError, TypeError):
            frame = Matrix.Identity(4)

    # ---- Phase 1: Object mode — datablocks only -------------------------
    _ensure_object_mode(context)

    rig_name = _rig_name(manifest)
    collection = bpy.data.collections.get(rig_name)
    if collection is None or collection not in list(context.scene.collection.children_recursive):
        collection = bpy.data.collections.new(rig_name)
        context.scene.collection.children.link(collection)
    _remove_previous_rig(collection)

    # ops.armature_add would depend on cursor, context overrides and the
    # active collection; direct datablock creation depends on nothing.
    arm_data = bpy.data.armatures.new(rig_name)
    arm_obj = bpy.data.objects.new(rig_name, arm_data)
    arm_obj["RIG_rig"] = True
    arm_obj["RIG_source"] = manifest.source_path or ""
    arm_obj["RIG_frame"] = [v for row in frame for v in row]
    arm_obj.show_in_front = True
    collection.objects.link(arm_obj)
    result.armature_object = arm_obj
    result.collection = collection

    helpers_coll = arm_data.collections.new(_HELPERS_COLLECTION)

    # ---- Phase 2: one Edit-mode session — every bone --------------------
    context.view_layer.objects.active = arm_obj
    arm_obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        edit_bones = arm_data.edit_bones
        for bp in plan.bones:
            eb = edit_bones.new(bp.bone_name)
            # A zero-length edit bone is silently deleted on mode exit, so
            # the bone gets a provisional tail before the matrix.
            eb.head = (0.0, 0.0, 0.0)
            eb.tail = (0.0, 1.0, 0.0)
            eb.matrix = frame @ _bone_rest_matrix(manifest, bp, unit_scale)
            eb.length = _bone_length(bp.group, unit_scale)
            # Connected bones ignore Limit Location entirely — a prismatic
            # joint dies silently — so no bone is ever connected.
            eb.use_connect = False
            if bp.parent_group_id is not None:
                eb.parent = edit_bones[result.bone_names[bp.parent_group_id]]
            result.bone_names[bp.group.id] = eb.name

            if bp.ball_def_name:
                # Swing-cone ball: the bone above is the USER HANDLE (free
                # rotation, no clamps). Geometry and child bones ride DEF,
                # which equals the handle inside the cone and is minimally
                # corrected onto the cone surface beyond it (constraints in
                # Phase 3). POLE anchors the cone axis at one handle-length
                # from the centre; GOAL rests at the handle's tail. All three
                # created BEFORE any child bone looks up bone_names[gid], so
                # children parent to DEF, never to the unclamped handle.
                result.ball_ctrl_names[bp.group.id] = eb.name
                parent_eb = eb.parent
                ctrl_matrix = eb.matrix.copy()
                ctrl_length = eb.length
                joint = bp.joint
                head = ctrl_matrix.translation.copy()
                cone_axis = (frame.to_3x3() @ Vector(joint.axis)).normalized()

                db = edit_bones.new(bp.ball_def_name)
                db.head = (0.0, 0.0, 0.0)
                db.tail = (0.0, 1.0, 0.0)
                db.matrix = ctrl_matrix
                db.length = ctrl_length
                db.use_connect = False
                db.parent = parent_eb
                helpers_coll.assign(db)
                result.bone_names[bp.group.id] = db.name

                pb_ = edit_bones.new(bp.ball_pole_name)
                pb_.head = (0.0, 0.0, 0.0)
                pb_.tail = (0.0, 1.0, 0.0)
                m = _frame_matrix(cone_axis, None, head + cone_axis * ctrl_length)
                pb_.matrix = m
                pb_.length = _HELPER_LENGTH_M * unit_scale
                pb_.use_connect = False
                pb_.parent = parent_eb
                helpers_coll.assign(pb_)
                result.ball_pole_names[bp.group.id] = pb_.name

                gb = edit_bones.new(bp.ball_goal_name)
                gb.head = (0.0, 0.0, 0.0)
                gb.tail = (0.0, 1.0, 0.0)
                m = ctrl_matrix.copy()
                ctrl_y = ctrl_matrix.col[1].to_3d().normalized()
                for i in range(3):
                    m[i][3] = head[i] + ctrl_y[i] * ctrl_length
                gb.matrix = m
                gb.length = _HELPER_LENGTH_M * unit_scale
                gb.use_connect = False
                gb.parent = parent_eb
                helpers_coll.assign(gb)
                result.ball_goal_names[bp.group.id] = gb.name

            c = bp.collapsed
            if c is not None and c.kind == "cone_spin":
                # Tangent cone on ONE grabbable bone (live corpus 15 cone3,
                # 2026-08-23): the bone above is the HANDLE — it slides on
                # the plane and rotates freely. DEF carries geometry and
                # children, following the handle with its axis clamped onto
                # the fixed-tilt ring (the ball template with a degenerate
                # band). POLE anchors the ring and RIDES the handle through
                # a local-space location copy — its rest orientation must
                # equal the handle's so the local channels map 1:1. FRM is
                # the STATIC plane frame the handle's on-plane clamp
                # measures in (a follower frame would cycle the depsgraph).
                result.ball_ctrl_names[bp.group.id] = eb.name
                parent_eb = eb.parent
                ctrl_matrix = eb.matrix.copy()
                ctrl_length = eb.length
                head = ctrl_matrix.translation.copy()
                normal = (frame.to_3x3()
                          @ Vector(c.carrier_joint.axis)).normalized()

                db = edit_bones.new(c.def_name)
                db.head = (0.0, 0.0, 0.0)
                db.tail = (0.0, 1.0, 0.0)
                db.matrix = ctrl_matrix
                db.length = ctrl_length
                db.use_connect = False
                db.parent = parent_eb
                helpers_coll.assign(db)
                result.bone_names[bp.group.id] = db.name

                fb = edit_bones.new(c.frame_name)
                fb.head = (0.0, 0.0, 0.0)
                fb.tail = (0.0, 1.0, 0.0)
                fb.matrix = _frame_matrix(normal, None, head)
                fb.length = _HELPER_LENGTH_M * unit_scale
                fb.use_connect = False
                fb.parent = parent_eb
                helpers_coll.assign(fb)
                result.cone_frame_names[bp.group.id] = fb.name

                pb_ = edit_bones.new(c.pole_name)
                pb_.head = (0.0, 0.0, 0.0)
                pb_.tail = (0.0, 1.0, 0.0)
                m = ctrl_matrix.copy()
                for i in range(3):
                    m[i][3] = head[i] + normal[i] * ctrl_length
                pb_.matrix = m
                pb_.length = _HELPER_LENGTH_M * unit_scale
                pb_.use_connect = False
                pb_.parent = parent_eb
                helpers_coll.assign(pb_)
                result.ball_pole_names[bp.group.id] = pb_.name

                gb = edit_bones.new(c.goal_name)
                gb.head = (0.0, 0.0, 0.0)
                gb.tail = (0.0, 1.0, 0.0)
                m = ctrl_matrix.copy()
                ctrl_y = ctrl_matrix.col[1].to_3d().normalized()
                for i in range(3):
                    m[i][3] = head[i] + ctrl_y[i] * ctrl_length
                gb.matrix = m
                gb.length = _HELPER_LENGTH_M * unit_scale
                gb.use_connect = False
                gb.parent = parent_eb
                helpers_coll.assign(gb)
                result.ball_goal_names[bp.group.id] = gb.name

        # Orbit-contact targets: a hidden bone at the orbit centre, riding
        # the parent body — the child bone's Limit Distance holds the
        # tangency radius against it.
        for bp in plan.bones:
            c = bp.collapsed
            if c is None or c.kind != "orbit_spin" or not c.helper_name:
                continue
            m = _frame_matrix(c.orbit_axis, None, c.orbit_center)
            for i in range(3):
                m[i][3] *= unit_scale
            eb = edit_bones.new(c.helper_name)
            eb.head = (0.0, 0.0, 0.0)
            eb.tail = (0.0, 1.0, 0.0)
            eb.matrix = frame @ m
            eb.length = _HELPER_LENGTH_M * unit_scale
            eb.use_connect = False
            if bp.parent_group_id is not None:
                eb.parent = edit_bones[result.bone_names[bp.parent_group_id]]
            helpers_coll.assign(eb)
            result.tangent_helper_names[bp.group.id] = eb.name

        for lplan in plan.loops:
            cj = lplan.closure_joint
            origin = cj.origin
            if origin is None:
                origin = _group_fallback_translation(
                    manifest, plan.bone_by_group[lplan.ik_tip_group].group)
            if cj.axis is not None:
                m = _frame_matrix(cj.axis, cj.secondary_axis, origin)
                for i in range(3):
                    m[i][3] *= unit_scale
            else:
                m = Matrix.Identity(4)
                for i in range(3):
                    m[i][3] = origin[i] * unit_scale
            eb = edit_bones.new(lplan.helper_name)
            eb.head = (0.0, 0.0, 0.0)
            eb.tail = (0.0, 1.0, 0.0)
            eb.matrix = frame @ m
            eb.length = _HELPER_LENGTH_M * unit_scale
            eb.use_connect = False
            eb.parent = edit_bones[result.bone_names[lplan.helper_parent_group]]
            helpers_coll.assign(eb)
            result.helper_names[lplan.loop.id] = eb.name

            # The effector carries the closure point rigidly on the driven
            # tip: its TAIL sits exactly at the closure origin, because the
            # tip bone's own tail is NOT the closure point — with bones along
            # the hinge axes a tail cannot even move in the mechanism plane,
            # and aiming IK at it left the solve dead (live corpus 06). The
            # tail, not the head: IK's use_tail=False actually re-targets the
            # owner's PARENT's tail, which is only the owner's head for
            # connected bones, so loops.py keeps use_tail on.
            eb = edit_bones.new(lplan.effector_name)
            eb.head = (0.0, 0.0, 0.0)
            eb.tail = (0.0, 1.0, 0.0)
            eb.matrix = frame @ m
            eb.length = _HELPER_LENGTH_M * unit_scale
            eb.translate(eb.head - eb.tail)   # land the tail on the closure
            eb.use_connect = False
            eb.parent = edit_bones[result.bone_names[lplan.ik_tip_group]]
            helpers_coll.assign(eb)
            result.effector_names[lplan.loop.id] = eb.name
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")

    helpers_coll.is_visible = False

    # Rails and patches are plain objects (Object mode work) and must exist
    # before Phase 3 hands them to the shrinkwrap constraints.
    contact_meshes = {}
    for bp in plan.bones:
        if bp.joint is None:
            continue
        obj = None
        if bp.joint.type == "path" and bp.joint.path_points:
            obj = _make_path_rail(collection, bp.joint, frame, unit_scale)
        elif bp.joint.type == "surface" and bp.joint.surface_triangles:
            obj = _make_surface_patch(collection, bp.joint, frame, unit_scale)
        if obj is not None:
            contact_meshes[bp.joint.id] = obj
            result.contact_mesh_names[bp.joint.id] = obj.name

    # ---- Phase 3: Pose mode — channels, constraints, drivers, IK --------
    bpy.ops.object.mode_set(mode="POSE")
    try:
        pose = arm_obj.pose
        # rotation_mode before anything else: the Limit Rotation euler_order
        # is copied from it, and switching it later re-interprets any value a
        # constraint or driver already wrote.
        for name in (list(result.bone_names.values())
                     + list(result.helper_names.values())
                     + list(result.effector_names.values())
                     + list(result.tangent_helper_names.values())
                     + list(result.ball_ctrl_names.values())
                     + list(result.ball_pole_names.values())
                     + list(result.ball_goal_names.values())
                     + list(result.cone_frame_names.values())):
            pose.bones[name].rotation_mode = "YXZ"

        for bp in plan.bones:
            pb = pose.bones[result.bone_names[bp.group.id]]
            pb["RIG_group"] = bp.group.id
            if bp.ball_def_name:
                pb["RIG_joint"] = bp.joint.id
                ctrl_pb = pose.bones[result.ball_ctrl_names[bp.group.id]]
                ctrl_pb["RIG_joint"] = bp.joint.id
                constraints.apply_ball_cone(
                    arm_obj, pb, ctrl_pb,
                    pose.bones[result.ball_goal_names[bp.group.id]],
                    pose.bones[result.ball_pole_names[bp.group.id]],
                    bp.joint)
                continue
            if bp.collapsed is not None and bp.collapsed.kind == "cone_spin":
                pb["RIG_joint"] = bp.collapsed.spin_joint.id
                ctrl_pb = pose.bones[result.ball_ctrl_names[bp.group.id]]
                ctrl_pb["RIG_joint"] = bp.collapsed.spin_joint.id
                constraints.apply_cone_spin(
                    arm_obj, pb, ctrl_pb,
                    pose.bones[result.ball_goal_names[bp.group.id]],
                    pose.bones[result.ball_pole_names[bp.group.id]],
                    pose.bones[result.cone_frame_names[bp.group.id]],
                    bp.collapsed.tilt, bp.collapsed.spin_joint.id)
                continue
            if bp.collapsed is not None:
                pb["RIG_joint"] = bp.collapsed.spin_joint.id
                target_name = result.tangent_helper_names.get(bp.group.id)
                constraints.apply_collapsed_contact(
                    pb, bp.collapsed, unit_scale,
                    orbit_target=arm_obj, orbit_subtarget=target_name)
                if target_name is not None:
                    hb = pose.bones[target_name]
                    hb["RIG_helper"] = bp.collapsed.spin_joint.id
                    hb.lock_location = [True, True, True]
                    hb.lock_rotation = [True, True, True]
                    hb.lock_scale = [True, True, True]
            elif bp.joint is not None:
                pb["RIG_joint"] = bp.joint.id
                constraints.apply_joint(
                    pb, bp.joint, unit_scale,
                    contact_mesh=contact_meshes.get(bp.joint.id))
                if (bp.joint.coupling is not None
                        and bp.joint.coupling.kind == "mirror"):
                    # The mirror-DRIVEN bone is entirely driver-owned: locks
                    # stop the user, the six sign-flip drivers still animate
                    # every channel through them.
                    pb.lock_location = [True, True, True]
                    pb.lock_rotation = [True, True, True]
            elif bp.group.grounded:
                constraints.lock_all(pb)
            else:
                constraints.unlock_all(pb)

        for lplan in plan.loops:
            # Helper and effector only ever move with their parents;
            # hand-posing either would silently detune the closure. Both
            # carry RIG_helper so parenting and matching skip them.
            for name in (result.helper_names[lplan.loop.id],
                         result.effector_names[lplan.loop.id]):
                pb = pose.bones[name]
                pb["RIG_helper"] = lplan.loop.id
                pb.lock_location = [True, True, True]
                pb.lock_rotation = [True, True, True]
                pb.lock_scale = [True, True, True]

        n_drivers, drv_warnings = drivers.build(
            arm_obj, manifest, plan, result.bone_names,
            unit_scale=unit_scale, context=context)
        result.warnings.extend(drv_warnings)

        n_loops, loop_warnings = loops.close_loops(
            arm_obj, plan, result.bone_names, result.helper_names,
            result.effector_names)
        result.warnings.extend(loop_warnings)
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")

    # A rail or patch is geometry of the joint's PARENT side — when that body
    # moves, the slot the child follows moves with it. Same bone-parent
    # matrix math as parenting.py: BONE parenting evaluates at the tail, so
    # the parent inverse is set against it and matrix_basis restores the
    # world placement exactly.
    if contact_meshes:
        context.view_layer.update()
        for bp in plan.bones:
            if bp.joint is None or bp.joint.id not in contact_meshes:
                continue
            obj = contact_meshes[bp.joint.id]
            bone_name = result.bone_names.get(bp.joint.parent_group)
            if bone_name is None:
                continue
            world_before = obj.matrix_world.copy()
            bone = arm_obj.data.bones[bone_name]
            p = (arm_obj.matrix_world
                 @ bone.matrix_local
                 @ Matrix.Translation((0.0, bone.length, 0.0)))
            obj.parent = arm_obj
            obj.parent_type = "BONE"
            obj.parent_bone = bone_name
            obj.matrix_parent_inverse = p.inverted()
            obj.matrix_basis = world_before
        context.view_layer.update()

    return result
