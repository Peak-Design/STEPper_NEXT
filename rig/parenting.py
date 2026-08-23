# SPDX-License-Identifier: GPL-3.0-or-later
"""Geometry attachment: matched objects -> bones, directly.

An earlier design put one GRP_ empty per rigid group between the bone and
the geometry. Dropped 2026-08-22: the empties bought nothing the RIG_*
object tags do not already provide (identity survives on the objects, not
on the middleman) and they cluttered the outliner. Legacy GRP_ empties are
still recognised and cleaned up by the rig rebuild (they carry RIG_rig).

BONE parenting evaluates at the bone TAIL, not the head, and against the
POSE matrix, so the parent matrix is computed deterministically as
    P = armature.matrix_world @ pose_bone.matrix @ Translation((0, length, 0))
and matrix_parent_inverse / matrix_basis are set so matrix_world is
byte-for-byte preserved. Guessing with ops or leaving Blender to compute
the inverse gives a rig where every part jumps by one bone length.

pose_bone.matrix, NOT bone.matrix_local: the two only agree at rest, and a
freshly built rig is not guaranteed to BE at rest — a limit constraint
whose range excludes the current pose clamps the bone the moment it
evaluates (live corpus 07, 2026-08-23: a mis-signed manifest limit shoved
the leaf bone 40° and the rest-matrix formula dragged the geometry with
it). The rest-matrix formula silently bakes any such clamp into the
geometry; the pose-matrix formula preserves the geometry no matter what
the constraints did, and report.posed_bones says loudly that a bone was
off rest while relinking.

Big-scene pattern: set ALL parents first, ONE view_layer.update(), then set
all matrices — the per-object update Blender does implicitly otherwise is
quadratic in scene size.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

try:
    import bpy
    from mathutils import Matrix
except ImportError:
    bpy = None
    Matrix = None

_DRIFT_TOL = 1e-6


@dataclass
class ParentReport:
    bone_parented: int = 0
    missing_groups: List[str] = field(default_factory=list)
    # (object name, drift in Blender units); collected, never raised —
    # aborting mid-scene would leave half the assembly re-parented.
    violations: List[Tuple[str, float]] = field(default_factory=list)
    # (bone name, metres its head sits from rest at relink time). A bone off
    # rest before anyone posed it means a constraint rejects the rest pose —
    # almost always a manifest limit whose value_at_rest lies outside its
    # own min/max. Geometry is preserved regardless; this is the tell.
    posed_bones: List[Tuple[str, float]] = field(default_factory=list)


def _bone_parent_matrix(arm_obj, bone_name):
    pb = arm_obj.pose.bones[bone_name]
    bone = arm_obj.data.bones[bone_name]
    return (arm_obj.matrix_world
            @ pb.matrix
            @ Matrix.Translation((0.0, bone.length, 0.0)))


def _rig_maps(arm_obj):
    bone_by_group = {}
    for pb in arm_obj.pose.bones:
        gid = pb.get("RIG_group")
        if gid and "RIG_helper" not in pb.keys():
            bone_by_group[gid] = pb.name
    geometry = {}
    for obj in bpy.data.objects:
        if obj.get("RIG_group_empty") or obj.get("RIG_rig"):
            continue    # legacy rig empties and the armature itself
        gid = obj.get("RIG_group")
        if gid:
            geometry.setdefault(gid, []).append(obj)
    return bone_by_group, geometry


def relink(context, arm_obj) -> ParentReport:
    """(Re-)parents everything the scene tags point at. Works from custom
    properties only — bone and object names are display labels that Blender
    rewrites on collision — so it runs identically after a file round-trip
    with no session state. Intended to run at rest pose: the parent matrix
    uses the bone's rest position, so a posed rig would record drift."""
    report = ParentReport()
    # The parent matrices read evaluated pose state (pose_bone.matrix
    # includes constraints), so the depsgraph must be current before
    # anything is captured.
    context.view_layer.update()
    bone_by_group, geometry = _rig_maps(arm_obj)

    plan = []  # (obj, bone_name, world_before)
    for gid in sorted(set(bone_by_group) | set(geometry)):
        bone_name = bone_by_group.get(gid)
        if bone_name is None:
            report.missing_groups.append(gid)
            continue
        for obj in geometry.get(gid, []):
            plan.append((obj, bone_name, obj.matrix_world.copy()))

    for bone_name in sorted({name for _, name, _ in plan}):
        pb = arm_obj.pose.bones[bone_name]
        rest = arm_obj.data.bones[bone_name].matrix_local
        delta = rest.inverted() @ pb.matrix
        # A limit clamp usually rotates about the bone head, so the head
        # barely moves — the rotation angle is the sensitive measure.
        off = max((pb.matrix.translation - rest.translation).length,
                  abs(delta.to_quaternion().angle))
        if off >= 1e-5:
            report.posed_bones.append((bone_name, off))
            print("[SWTB relink] bone %s sits off its rest pose while "
                  "relinking (%.4f rad/m) — a constraint rejects the rest "
                  "pose (check that joint's limits vs value_at_rest); "
                  "geometry keeps its place regardless" % (bone_name, off))

    for obj, bone_name, _ in plan:
        obj.parent = arm_obj
        obj.parent_type = "BONE"
        obj.parent_bone = bone_name
        obj["RIG_parent_mode"] = "BONE"
        report.bone_parented += 1

    context.view_layer.update()

    for obj, bone_name, world_before in plan:
        p = _bone_parent_matrix(arm_obj, bone_name)
        obj.matrix_parent_inverse = p.inverted()
        # world = P @ parent_inverse @ basis, and parent_inverse is P^-1,
        # so restoring the world transform is exactly basis = world_before.
        obj.matrix_basis = world_before

    context.view_layer.update()

    for obj, _, world_before in plan:
        drift = (obj.matrix_world.translation - world_before.translation).length
        if drift >= _DRIFT_TOL:
            report.violations.append((obj.name, drift))

    return report
