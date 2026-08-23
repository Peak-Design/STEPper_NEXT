# SPDX-License-Identifier: GPL-3.0-or-later
"""Joint-graph planning: manifest -> RigPlan.

No bpy import — this module runs under plain Python in CI. Everything that
can make the generated rig wrong is decided and validated here, before a
single Blender datablock exists. A muted or zero-influence constraint still
counts as a depsgraph dependency in Blender, so a dependency cycle found
here means "do not create the rig", never "create it disabled".

The exporter already cut every graph cycle (SCHEMA.md: loops are pre-cut,
consumers must not re-derive them). This module only verifies the cut is
consistent and turns it into build orders.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from .manifest import Joint, Loop, Manifest, ManifestError, RigidGroup

# Blender truncates datablock names at 63 bytes and dedups with ".001";
# names are display labels only, so plans keep them short and every module
# addresses bones through the RigPlan / build maps, never by parsing names.
_MAX_NAME = 55


@dataclass
class CollapsedContact:
    """A carrier chain folded into ONE posable bone. The exporter decomposes
    a tangent/contact mate into carrier joint + spin joint through a
    component-less group; posing that chain means driving the CARRIER, which
    nobody thinks in ("rotate the helper to slide the puck"). The collapse
    gives the child bone the chain's combined freedom directly — locks in a
    contact-aligned frame, plus a Limit Distance for orbit contacts — so the
    puck is grabbed and moved like in SolidWorks (Oscar's ask, 2026-08-23).

    kinds: planar_spin (cylinder on plane: slide in plane, yaw about the
    normal, spin about own axis), orbit_spin (rim-tangent discs: drag around
    the base at the tangency radius, spin free), planar_ball (sphere/vertex
    on plane), slide_ball (vertex on line), cone_spin (cone tangent on a
    plane: like planar_spin but the spin axis is TILTED out of the plane by
    the cone's half-angle, so channel locks cannot hold it — the ball
    template's chord clamp pins the axis to its fixed-tilt ring instead,
    live corpus 15 cone3, 2026-08-23)."""
    kind: str
    carrier_joint: Joint
    spin_joint: Joint
    orbit_center: Optional[List[float]] = None   # metres (orbit_spin only)
    orbit_axis: Optional[List[float]] = None
    orbit_radius: float = 0.0                    # metres
    helper_name: str = ""                        # Limit Distance target bone
    # cone_spin only: the fixed angle between the spin axis and the plane
    # NORMAL (the ring the DEF clamp holds), plus the hidden template bones.
    tilt: float = 0.0                            # radians
    def_name: str = ""
    pole_name: str = ""
    goal_name: str = ""
    frame_name: str = ""                         # plane-frame custom space


@dataclass
class BonePlan:
    group: RigidGroup
    bone_name: str
    parent_group_id: Optional[str] = None
    joint: Optional[Joint] = None
    collapsed: Optional[CollapsedContact] = None
    # The one grounded root: named after the assembly, resting AT the
    # assembly origin. A grounded group's own frame is whichever member the
    # exporter happened to anchor it to (live 2026-08-23: the flexible-sub
    # "baseplate" bone landed on the hinge), and static geometry needs no
    # per-group placement — the origin is the WYSIWYG spot.
    root: bool = False
    # Swing-cone ball template (joint.axis = cone axis, secondary = the
    # child direction the limit band constrains): bone_name is the user
    # handle, DEF carries the geometry through the exact swing clamp, POLE
    # anchors the cone axis, GOAL is the clamped aim point. Empty for every
    # other joint. The template's dependencies (DEF -> ctrl/GOAL,
    # GOAL -> ctrl/POLE, all under one parent bone) are acyclic by
    # construction, so the pre-flight needs no extra nodes for them.
    ball_def_name: str = ""
    ball_pole_name: str = ""
    ball_goal_name: str = ""
    # Mirror pairs (coupling kind "mirror"): BOTH bones of the pair rest
    # with the SAME plane-aligned orientation (local +Y = this normal) at
    # their own positions — equal rest axes reduce the reflection to
    # per-channel sign flips, which is what makes the six drivers exact.
    mirror_normal: Optional[List[float]] = None


@dataclass
class LoopPlan:
    loop: Loop
    closure_joint: Joint
    helper_name: str
    # The helper bone rides the driver-side chain so the closure point moves
    # with the posed side; the IK chain on the driven side follows it. The
    # effector bone is the same closure point rigid on the driven tip — the
    # IK constraint lives on it because the tip bone's own tail is NOT the
    # closure point (live corpus 06: bones point along the hinge axes, so
    # tails cannot even move in the mechanism plane and the solve went dead).
    effector_name: str
    helper_parent_group: str
    ik_tip_group: str
    driven_chain: List[str] = field(default_factory=list)   # tip first, ancestor excluded
    driver_chain: List[str] = field(default_factory=list)   # driver end first, ancestor excluded
    chain_count: int = 0


@dataclass
class RigPlan:
    manifest: Manifest
    bones: List[BonePlan] = field(default_factory=list)     # parents always before children
    bone_by_group: Dict[str, BonePlan] = field(default_factory=dict)
    joint_group: Dict[str, str] = field(default_factory=dict)  # tree joint id -> articulating group id
    loops: List[LoopPlan] = field(default_factory=list)
    grounded_groups: List[str] = field(default_factory=list)
    free_groups: List[str] = field(default_factory=list)    # unparented, not grounded
    collapsed_carriers: List[str] = field(default_factory=list)  # groups with no bone
    warnings: List[str] = field(default_factory=list)


class _UnionFind:
    def __init__(self, ids):
        self._parent = {i: i for i in ids}

    def find(self, i):
        root = i
        while self._parent[root] != root:
            root = self._parent[root]
        while self._parent[i] != root:
            self._parent[i], i = root, self._parent[i]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return False
        self._parent[ra] = rb
        return True


def _unique_name(base: str, taken: Set[str], suffix: str) -> str:
    name = base[:_MAX_NAME] if len(base) > _MAX_NAME else base
    if name in taken:
        name = "{}.{}".format(name[:_MAX_NAME - len(suffix) - 1], suffix)
    taken.add(name)
    return name


def _path_to_root(group_id: str, parent_of: Dict[str, Tuple[str, Joint]]) -> List[str]:
    path = [group_id]
    while path[-1] in parent_of:
        path.append(parent_of[path[-1]][0])
    return path


def _branches(parent_of, end_a, end_b):
    """Splits the tree path between two groups at their lowest common
    ancestor. Each branch is (groups from the end up to but excluding the
    ancestor, the joints along it)."""
    path_a = _path_to_root(end_a, parent_of)
    path_b = _path_to_root(end_b, parent_of)
    on_b = set(path_b)
    ancestor = None
    for g in path_a:
        if g in on_b:
            ancestor = g
            break
    if ancestor is None:
        return None

    def cut(path):
        groups = path[:path.index(ancestor)]
        joints = [parent_of[g][1] for g in groups]
        return groups, joints

    return ancestor, cut(path_a), cut(path_b)


def _v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_norm(a):
    return _v_dot(a, a) ** 0.5


def _unit(a):
    n = _v_norm(a)
    return [a[0] / n, a[1] / n, a[2] / n] if n > 1e-12 else None


def swing_cone(joint: Optional[Joint]) -> bool:
    """A ball whose limit is a swing cone about a mate-defined axis. The
    exporter ships the cone frame in axis (parent-fixed cone axis, sign
    meaningful — it says which way the cone opens) and secondary_axis (the
    child-fixed direction the [min, max] band constrains). A limited ball
    WITHOUT an axis is a legacy manifest; it falls back to the Euler box."""
    return (joint is not None and joint.type == "ball"
            and joint.axis is not None and joint.secondary_axis is not None
            and joint.rotation_limit is not None)


_COLLAPSE_KINDS = {
    ("planar", "revolute"): "planar_spin",
    ("revolute", "revolute"): "orbit_spin",
    ("planar", "ball"): "planar_ball",
    ("prismatic", "ball"): "slide_ball",
}


def _collapse_carriers(plan: RigPlan, groups, parent_of) -> Dict[str, CollapsedContact]:
    """Folds P -> carrier -> K chains into P -> K where the carrier is a
    component-less contact link matching a known pattern. Mutates parent_of
    and joint_group; returns child group id -> collapse spec. Anything with
    limits, couplings, loop membership, or geometry the patterns cannot
    absorb keeps its carrier bone — correct beats convenient."""
    children_of = {}
    for child, (parent, _) in parent_of.items():
        children_of.setdefault(parent, []).append(child)

    in_loops = set()
    for lplan in plan.loops:
        in_loops.update(lplan.driven_chain)
        in_loops.update(lplan.driver_chain)
        in_loops.add(lplan.helper_parent_group)
        in_loops.add(lplan.ik_tip_group)
        in_loops.add(lplan.closure_joint.parent_group)
        in_loops.add(lplan.closure_joint.child_group)

    coupling_drivers = set()
    for j in plan.manifest.joints:
        if j.coupling is not None and j.coupling.driver_joint:
            coupling_drivers.add(j.coupling.driver_joint)

    specs = {}
    for cid in sorted(groups):
        group = groups[cid]
        if group.components or group.grounded:
            continue
        if cid not in parent_of or cid in in_loops:
            continue
        kids = children_of.get(cid, [])
        if len(kids) != 1:
            continue
        kid = kids[0]
        parent_id, j1 = parent_of[cid]
        _, j2 = parent_of[kid]
        kind = _COLLAPSE_KINDS.get((j1.type, j2.type))
        if kind is None:
            continue
        if any(j.rotation_limit or j.translation_limit or j.coupling
               for j in (j1, j2)):
            continue
        if j1.id in coupling_drivers or j2.id in coupling_drivers:
            continue
        if j1.axis is None or (kind in ("planar_spin", "orbit_spin")
                               and j2.axis is None):
            continue

        spec = CollapsedContact(kind=kind, carrier_joint=j1, spin_joint=j2)
        if kind == "orbit_spin":
            n = _unit(j1.axis)
            a = _unit(j2.axis)
            if n is None or a is None or abs(_v_dot(n, a)) < 0.999:
                continue   # skewed axes: the chain is not a plain orbit
            if j1.origin is None or j2.origin is None:
                continue
            d = _v_sub(j2.origin, j1.origin)
            along = _v_dot(d, n)
            radial = _v_sub(d, [n[0] * along, n[1] * along, n[2] * along])
            radius = _v_norm(radial)
            if radius < 1e-6:
                continue   # concentric, not tangent — not this pattern
            # The Limit Distance target must sit IN the child's orbit plane:
            # the constraint clamps to a SPHERE, and only with zero axial
            # offset does sphere ∩ plane-lock equal the tangency circle at
            # the radial distance. So the centre is the orbit-axis point at
            # the child's height, not the carrier origin itself.
            spec.orbit_center = [j1.origin[0] + n[0] * along,
                                 j1.origin[1] + n[1] * along,
                                 j1.origin[2] + n[2] * along]
            spec.orbit_axis = list(n)
            spec.orbit_radius = radius
        elif kind == "planar_spin":
            n = _unit(j1.axis)
            a = _unit(j2.axis)
            if n is None or a is None:
                continue
            if abs(_v_dot(n, a)) > 0.1:
                # The spin axis is tilted OUT of the plane: a tangent CONE
                # (live corpus 15 cone3, 2026-08-23). Channel locks cannot
                # hold a tilted precession, so this collapses through the
                # ball template instead — the DEF axis pinned to its
                # fixed-tilt ring around the plane normal.
                kind = "cone_spin"
                spec.kind = kind
                dot = max(-1.0, min(1.0, _v_dot(n, a)))
                spec.tilt = math.acos(dot)

        # Rewire: the child hangs from the carrier's parent on the SPIN
        # joint (its axis and origin are the child's own), the carrier
        # leaves the tree and gets no bone.
        parent_of[kid] = (parent_id, j2)
        del parent_of[cid]
        plan.joint_group.pop(j1.id, None)
        plan.collapsed_carriers.append(cid)
        specs[kid] = spec
    return specs


def _assert_acyclic(edges: Dict[str, Set[str]], labels: Dict[str, str]):
    """Raises on the first directed cycle. Iterative DFS — assembly trees can
    be deep enough to hit the interpreter recursion limit."""
    state = {}  # node -> 1 while on stack, 2 when finished
    for start in sorted(edges):
        if state.get(start):
            continue
        stack = [(start, iter(sorted(edges.get(start, ()))))]
        state[start] = 1
        while stack:
            node, it = stack[-1]
            advanced = False
            for nxt in it:
                if state.get(nxt) == 1:
                    chain = [n for n, _ in stack]
                    cycle = chain[chain.index(nxt):] + [nxt]
                    raise ManifestError(
                        "rig dependency cycle: "
                        + " -> ".join(labels.get(n, n) for n in cycle))
                if state.get(nxt) is None:
                    state[nxt] = 1
                    stack.append((nxt, iter(sorted(edges.get(nxt, ())))))
                    advanced = True
                    break
            if not advanced:
                state[node] = 2
                stack.pop()


def build(manifest: Manifest) -> RigPlan:
    """Plans the rig. Raises manifest.ManifestError on any inconsistency the
    exporter contract forbids, and on any dependency cycle the generated rig
    would hand to Blender's depsgraph."""
    plan = RigPlan(manifest=manifest)
    groups = manifest.group_by_id()
    joints = manifest.joint_by_id()

    closure_ids = set()
    for lp in manifest.loops:
        closure_ids.add(lp.closure_joint)

    # Free joints mark under-mated pairs; SCHEMA.md says leave them
    # unparented rather than silently fixing them, so they are not tree
    # edges — EXCEPT the two halves of a mirror pair: a free joint that
    # carries or drives a mirror coupling is deliberately ground-rooted
    # (the whole relation is the reflection, live corpus 14 sym4,
    # 2026-08-23), so both parent like any tree edge.
    mirror_driver_ids = set()
    mirror_normal_of = {}
    for j in manifest.joints:
        c = j.coupling
        if c is None or c.kind != "mirror" or c.mirror_plane_normal is None:
            continue
        mirror_normal_of[j.child_group] = list(c.mirror_plane_normal)
        if c.driver_joint:
            mirror_driver_ids.add(c.driver_joint)
            drv = joints.get(c.driver_joint)
            if drv is not None:
                mirror_normal_of[drv.child_group] = list(c.mirror_plane_normal)
    tree_joints = [j for j in manifest.joints
                   if (j.type != "free"
                       or j.id in mirror_driver_ids
                       or (j.coupling is not None and j.coupling.kind == "mirror"))
                   and j.id not in closure_ids]

    parent_of = {}  # child group id -> (parent group id, joint)
    forest = _UnionFind(groups.keys())
    for j in tree_joints:
        if j.parent_group == j.child_group:
            raise ManifestError(
                "joint {}: parent and child are both {}".format(j.id, j.parent_group))
        if groups[j.child_group].grounded:
            raise ManifestError(
                "joint {}: child {} is grounded; the exporter's spanning tree "
                "roots at grounded groups".format(j.id, j.child_group))
        if j.child_group in parent_of:
            raise ManifestError(
                "group {} has two tree parents (joints {} and {}); the "
                "exporter should have cut one as a loop".format(
                    j.child_group, parent_of[j.child_group][1].id, j.id))
        if not forest.union(j.child_group, j.parent_group):
            raise ManifestError(
                "joint {} closes a cycle the exporter did not declare as a "
                "loop".format(j.id))
        parent_of[j.child_group] = (j.parent_group, j)
        plan.joint_group[j.id] = j.child_group

    # Loop verification: a closure joint must be a genuine non-tree edge —
    # its endpoints already connected through the tree — and the tree path
    # between them plus the closure must be exactly the declared members.
    for lp in manifest.loops:
        cj = joints[lp.closure_joint]
        if forest.find(cj.parent_group) != forest.find(cj.child_group):
            raise ManifestError(
                "loop {}: closure joint {} bridges two disconnected trees — "
                "it is a tree edge, not a loop closure".format(lp.id, cj.id))
        split = _branches(parent_of, cj.parent_group, cj.child_group)
        if split is None:
            raise ManifestError(
                "loop {}: no tree path between {} and {}".format(
                    lp.id, cj.parent_group, cj.child_group))
        ancestor, (groups_p, joints_p), (groups_c, joints_c) = split
        path_ids = {j.id for j in joints_p} | {j.id for j in joints_c} | {cj.id}
        if path_ids != set(lp.member_joints):
            raise ManifestError(
                "loop {}: member_joints {} do not match the tree path plus "
                "closure {}".format(lp.id, sorted(lp.member_joints), sorted(path_ids)))

        driver_side = None
        if lp.suggested_driver_joint:
            if any(j.id == lp.suggested_driver_joint for j in joints_p):
                driver_side = "p"
            elif any(j.id == lp.suggested_driver_joint for j in joints_c):
                driver_side = "c"
            else:
                plan.warnings.append(
                    "loop {}: suggested driver joint {} is not on the loop "
                    "path; falling back to the closure orientation".format(
                        lp.id, lp.suggested_driver_joint))
        if driver_side is None:
            # The exporter's closure orientation is the only stable default:
            # parent side drives, child side is IK-solved.
            driver_side = "p"
        # The IK chain must contain at least one bone; when the chosen driven
        # side is the ancestor itself, the sides swap regardless of the
        # suggestion.
        driven_groups = groups_c if driver_side == "p" else groups_p
        if not driven_groups:
            driver_side = "c" if driver_side == "p" else "p"
            driven_groups = groups_c if driver_side == "p" else groups_p
            plan.warnings.append(
                "loop {}: driven side was empty; driver/driven sides "
                "swapped".format(lp.id))
        if not driven_groups:
            raise ManifestError(
                "loop {}: closure joint {} connects a group to its own "
                "ancestor chain on both sides".format(lp.id, cj.id))
        if driver_side == "p":
            driver_groups = groups_p
            helper_parent = cj.parent_group
            ik_tip = cj.child_group
        else:
            driver_groups = groups_c
            helper_parent = cj.child_group
            ik_tip = cj.parent_group
        plan.loops.append(LoopPlan(
            loop=lp,
            closure_joint=cj,
            helper_name="",
            effector_name="",
            helper_parent_group=helper_parent,
            ik_tip_group=ik_tip,
            driven_chain=list(driven_groups),
            driver_chain=list(driver_groups),
            chain_count=len(driven_groups),
        ))

    # Contact carrier chains fold into single posable bones AFTER loop
    # verification (a carrier inside a loop keeps its bone — IK needs the
    # chain) and BEFORE the dependency pre-flight, which must model the rig
    # actually generated.
    collapse_specs = _collapse_carriers(plan, groups, parent_of)
    skipped = set(plan.collapsed_carriers)

    # Pre-flight dependency check over the rig that WILL be generated:
    # child bone -> parent bone, IK chain bone -> helper target, helper ->
    # its parent bone, driven channel -> driver source bone. Runs before any
    # bpy mutation so a cycle aborts with the scene untouched.
    def bone_node(gid):
        return "bone:" + gid

    edges = {}
    labels = {}
    for g in groups.values():
        if g.id in skipped:
            continue
        edges.setdefault(bone_node(g.id), set())
        labels[bone_node(g.id)] = "bone {} ({})".format(g.id, g.name)
    for child, (parent, j) in parent_of.items():
        edges[bone_node(child)].add(bone_node(parent))
    for lplan in plan.loops:
        hnode = "helper:" + lplan.loop.id
        labels[hnode] = "helper for loop " + lplan.loop.id
        edges.setdefault(hnode, set())
        edges[hnode].add(bone_node(lplan.helper_parent_group))
        enode = "effector:" + lplan.loop.id
        labels[enode] = "effector for loop " + lplan.loop.id
        edges.setdefault(enode, set())
        edges[enode].add(bone_node(lplan.ik_tip_group))   # its parent bone
        edges[enode].add(hnode)                           # its IK target
        for gid in lplan.driven_chain:
            edges[bone_node(gid)].add(hnode)
    for j in manifest.joints:
        if j.coupling is None or j.id not in plan.joint_group:
            continue
        driver_id = j.coupling.driver_joint
        if j.coupling.kind == "screw":
            # The screw self-driver reads its own bone's location to drive
            # its rotation; distinct channels on one bone are acyclic at
            # Blender's per-channel depsgraph granularity, so no edge.
            continue
        if driver_id is None:
            plan.warnings.append(
                "joint {}: {} coupling without a driver joint; the driver "
                "will be skipped".format(j.id, j.coupling.kind))
            continue
        if driver_id not in plan.joint_group:
            plan.warnings.append(
                "joint {}: coupling driver {} is not a tree joint; the "
                "driver will be skipped".format(j.id, driver_id))
            continue
        edges[bone_node(plan.joint_group[j.id])].add(
            bone_node(plan.joint_group[driver_id]))
    _assert_acyclic(edges, labels)

    # Build order: grounded roots, then free roots, then depth-first children,
    # every list sorted by group id so re-runs name and order identically.
    roots = [g for g in groups.values()
             if g.id not in parent_of and g.id not in skipped]
    for g in roots:
        if g.grounded:
            plan.grounded_groups.append(g.id)
        else:
            plan.free_groups.append(g.id)
    plan.grounded_groups.sort()
    plan.free_groups.sort()

    children = {}
    for child, (parent, j) in parent_of.items():
        children.setdefault(parent, []).append(child)
    for kids in children.values():
        kids.sort()

    taken_names = set()
    order = list(plan.grounded_groups) + list(plan.free_groups)
    stack = list(reversed(order))
    # The first grounded root becomes THE root: named after the assembly,
    # at the assembly origin (see BonePlan.root). Extra grounded islands
    # (two unconnected fixed components) keep their group identity.
    root_gid = plan.grounded_groups[0] if plan.grounded_groups else None
    assembly_stem = manifest.step_file.replace("\\", "/").rsplit("/", 1)[-1]
    if "." in assembly_stem:
        assembly_stem = assembly_stem.rsplit(".", 1)[0]
    while stack:
        gid = stack.pop()
        group = groups[gid]
        parent_id, joint = parent_of.get(gid, (None, None))
        is_root = gid == root_gid
        bp = BonePlan(
            group=group,
            bone_name=_unique_name(
                assembly_stem if is_root and assembly_stem else group.name,
                taken_names, group.id),
            parent_group_id=parent_id,
            joint=joint,
            collapsed=collapse_specs.get(gid),
            root=is_root,
            mirror_normal=mirror_normal_of.get(gid),
        )
        if swing_cone(joint):
            bp.ball_def_name = _unique_name("DEF_" + bp.bone_name, taken_names, gid)
            bp.ball_pole_name = _unique_name("POLE_" + bp.bone_name, taken_names, gid)
            bp.ball_goal_name = _unique_name("GOAL_" + bp.bone_name, taken_names, gid)
        plan.bones.append(bp)
        plan.bone_by_group[gid] = bp
        for child in reversed(children.get(gid, [])):
            stack.append(child)

    if len(plan.bones) != len(groups) - len(skipped):
        missing = sorted(set(groups) - set(plan.bone_by_group) - skipped)
        raise ManifestError(
            "groups {} are unreachable from any root; the tree walk lost "
            "them".format(missing))

    for lplan in plan.loops:
        lplan.helper_name = _unique_name(
            "HLP_" + lplan.loop.id, taken_names, lplan.loop.id)
        lplan.effector_name = _unique_name(
            "EFF_" + lplan.loop.id, taken_names, lplan.loop.id)
    for gid in sorted(collapse_specs):
        spec = collapse_specs[gid]
        if spec.kind == "orbit_spin":
            spec.helper_name = _unique_name(
                "TGT_" + plan.bone_by_group[gid].bone_name, taken_names, gid)
        elif spec.kind == "cone_spin":
            base = plan.bone_by_group[gid].bone_name
            spec.def_name = _unique_name("DEF_" + base, taken_names, gid)
            spec.pole_name = _unique_name("POLE_" + base, taken_names, gid)
            spec.goal_name = _unique_name("GOAL_" + base, taken_names, gid)
            spec.frame_name = _unique_name("FRM_" + base, taken_names, gid)

    return plan
