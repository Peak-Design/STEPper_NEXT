# SPDX-License-Identifier: GPL-3.0-or-later
"""Geometry matching: manifest components -> scene objects.

Blender object names are display labels — 63-byte truncation and ".001"
dedup make them useless as identity — so nothing here matches on obj.name
except the last-resort fuzzy step, which exists for foreign importers that
wrote no STEP_* properties at all.

The five-step cascade runs strictly in order, each step only over the
still-unmatched components and still-unclaimed objects. Results are written
back as obj["RIG_component_id"] / obj["RIG_group"], and a re-run pre-seeds
from those tags, so matching is idempotent.
"""

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .manifest import Component, Manifest

# Importable under plain Python; every entry point that needs Blender
# checks for bpy itself.
try:
    import bpy
except ImportError:
    bpy = None

_TRANSLATION_TOL = 1e-4   # metres
_ROTATION_TOL = 1e-3      # radians
_BBOX_REL_TOL = 0.01

_METHOD_NAMES = {
    0: "existing",
    1: "exact",
    2: "path",
    3: "transform",
    4: "bbox",
    5: "fuzzy",
}


@dataclass
class StepKey:
    name: Optional[str] = None
    uuid: Optional[int] = None
    parent: Optional[int] = None
    file: Optional[str] = None
    applied_scale: Optional[float] = None
    instance_of: Optional[str] = None


@dataclass
class MatchEntry:
    component_id: str
    object_name: str
    step: int
    confidence: str


@dataclass
class MatchReport:
    matched: List[MatchEntry] = field(default_factory=list)
    ambiguous: List[Tuple[str, List[str]]] = field(default_factory=list)
    unmatched: List[str] = field(default_factory=list)
    unclaimed_objects: List[str] = field(default_factory=list)
    # The scene-frame transform: manifest coordinates (Blender units) ->
    # where the geometry actually sits. Identity when the import kept the
    # manifest's Z-up frame; a rotation when the user imported with another
    # up axis. rig_build applies it to every bone it places.
    frame_rows: Optional[List[List[float]]] = None
    frame_agree: int = 0


def get_step_key(obj) -> StepKey:
    """The only place STEP_* custom properties are touched. Foreign importers
    (no STEPper NEXT metadata) come back as an all-None key and simply fall
    through to the fuzzy step."""
    key = StepKey()
    try:
        key.name = obj.get("STEP_name")
        key.uuid = obj.get("STEP_uuid")
        key.parent = obj.get("STEP_parent")
        key.file = obj.get("STEP_file")
        key.applied_scale = obj.get("STEP_applied_scale")
        key.instance_of = obj.get("STEP_instance_of")
    except (AttributeError, TypeError):
        pass
    return key


def _norm_path(path: str) -> str:
    parts = [p for p in str(path).replace("\\", "/").split("/") if p]
    return "/".join(parts)


def _strip_instance_suffix(segment: str) -> str:
    return re.sub(r"-\d+$", "", segment)


def _loose_path(path: str) -> str:
    return "/".join(_strip_instance_suffix(p) for p in _norm_path(path).split("/") if p)


def _fuzzy_label(text: str) -> str:
    # ".NNN" is Blender's duplicate suffix, "-N" is the CAD instance suffix;
    # both are identity noise for a last-resort name comparison.
    label = str(text)
    while True:
        stripped = re.sub(r"\.\d+$", "", label)
        stripped = re.sub(r"-\d+$", "", stripped)
        if stripped == label:
            break
        label = stripped
    return label.casefold()


def _occurrence_path(obj, by_uuid: Dict[Tuple[Optional[str], int], object]) -> Optional[str]:
    """Rebuilds the STEP occurrence path from STEP_uuid/STEP_parent chains.
    STEPper's artificial root node (parent -1, labelled '<file>.empties') is
    not part of the STEP tree and never enters the path. FLAT/TREE imports
    have no ancestor objects, so the chain degrades to the leaf name and the
    exact-path step simply fails over to the later steps."""
    segments = []
    cur = obj
    for _ in range(256):
        key = get_step_key(cur)
        if key.uuid is None:
            break
        if key.parent is not None and key.parent == -1:
            break
        segments.append(key.name or "")
        if key.parent is None:
            break
        cur = by_uuid.get((key.file, key.parent))
        if cur is None:
            break
    if not segments:
        return None
    segments.reverse()
    return "/".join(segments)


def _matrix_rows(obj) -> List[List[float]]:
    return [[float(v) for v in row] for row in obj.matrix_world]


def _scene_scale() -> float:
    if bpy is None:
        return 1.0
    try:
        scale = float(bpy.context.scene.unit_settings.scale_length)
    except (AttributeError, TypeError):
        return 1.0
    return scale if scale > 0.0 else 1.0


def _rotation_columns(rows) -> Optional[List[List[float]]]:
    """Unit-length rotation columns of a 4x4, scale divided out. A degenerate
    column (STEP_applied_scale left in the matrix can be 0 only through
    corruption) disqualifies the object from the transform step."""
    cols = []
    for c in range(3):
        v = [rows[0][c], rows[1][c], rows[2][c]]
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n < 1e-12:
            return None
        cols.append([x / n for x in v])
    return cols


def _rotation_angle(cols_a, cols_b) -> float:
    # trace(A^T B) via column dot products; acos clamped against fp drift.
    trace = 0.0
    for c in range(3):
        trace += (cols_a[c][0] * cols_b[c][0]
                  + cols_a[c][1] * cols_b[c][1]
                  + cols_a[c][2] * cols_b[c][2])
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))


# ── Scene-frame estimation ──────────────────────────────────────────────
# The manifest is Z-up SolidWorks global; the importer may have rotated the
# geometry (STEPper's up-axis option, or any foreign importer's convention).
# Comparing manifest transforms against object transforms directly would
# then fail everywhere — and a rig built in the manifest frame would not
# touch the geometry. The orientation-independent matches (steps 0–2, name
# and path based) anchor an estimate of the frame transform F with
# obj.matrix_world ≈ F @ component_transform; the transform step and the
# rig builder both work under F afterwards.

def _normalized_rot(rows):
    cols = _rotation_columns(rows)
    if cols is None:
        return None
    return [[cols[c][r] for c in range(3)] for r in range(3)]


def _rot_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot_t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _rot_apply(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _snap_frame_rot(r):
    """Up-axis conversions are exact ±90°/180° rotations; snapping removes
    the fp noise the anchor pair injected. Anything that does not snap to a
    legal right-handed axis swap comes back unchanged."""
    snapped = []
    for i in range(3):
        row = []
        for j in range(3):
            v = r[i][j]
            if abs(v) < 0.02:
                row.append(0.0)
            elif abs(v - 1.0) < 0.02:
                row.append(1.0)
            elif abs(v + 1.0) < 0.02:
                row.append(-1.0)
            else:
                return r
        snapped.append(row)
    for i in range(3):
        if sum(abs(x) for x in snapped[i]) != 1.0:
            return r
        if sum(abs(snapped[j][i]) for j in range(3)) != 1.0:
            return r
    det = (snapped[0][0] * (snapped[1][1] * snapped[2][2] - snapped[1][2] * snapped[2][1])
           - snapped[0][1] * (snapped[1][0] * snapped[2][2] - snapped[1][2] * snapped[2][0])
           + snapped[0][2] * (snapped[1][0] * snapped[2][1] - snapped[1][1] * snapped[2][0]))
    if abs(det - 1.0) > 1e-9:
        return r
    return snapped


def _frame_from_rot(rot, t=(0.0, 0.0, 0.0)):
    return [
        [rot[0][0], rot[0][1], rot[0][2], t[0]],
        [rot[1][0], rot[1][1], rot[1][2], t[1]],
        [rot[2][0], rot[2][1], rot[2][2], t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def identity_frame():
    return _frame_from_rot([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


# The up-axis conversions an importer plausibly applied, tried when no
# name/path anchor exists: Z-up kept, Y-up both ways, and upside down.
_CANDIDATE_ROTS = (
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],   # +90° about X
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],   # −90° about X
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],  # 180° about X
)


def apply_frame(frame, rows):
    out = []
    for i in range(4):
        out.append([sum(frame[i][k] * rows[k][j] for k in range(4)) for j in range(4)])
    return out


def _component_rows_units(component: Component, unit_scale: float):
    rows = [list(component.transform[i]) for i in range(4)]
    for i in range(3):
        rows[i][3] *= unit_scale
    return rows


def _frame_from_anchor(obj_rows, comp_rows):
    r_obj = _normalized_rot(obj_rows)
    r_comp = _normalized_rot(comp_rows)
    if r_obj is None or r_comp is None:
        return None
    rot = _snap_frame_rot(_rot_mul(r_obj, _rot_t(r_comp)))
    rt = _rot_apply(rot, [comp_rows[i][3] for i in range(3)])
    t = [obj_rows[i][3] - rt[i] for i in range(3)]
    t = [0.0 if abs(v) < 1e-7 else v for v in t]
    return _frame_from_rot(rot, t)


def _frame_agrees(frame, obj_rows, comp_rows, scene_scale,
                  t_tol=_TRANSLATION_TOL, r_tol=_ROTATION_TOL) -> bool:
    pred = apply_frame(frame, comp_rows)
    for i in range(3):
        if abs(obj_rows[i][3] - pred[i][3]) * scene_scale > t_tol:
            return False
    a = _rotation_columns(obj_rows)
    b = _rotation_columns(pred)
    if a is None or b is None:
        return False
    return _rotation_angle(a, b) <= r_tol


def estimate_frame(anchor_pairs, scene_scale):
    """anchor_pairs: (obj_rows, comp_rows_units) from orientation-independent
    matches. Each anchor proposes a frame; the one most anchors agree with
    wins — a single mis-tagged object cannot hijack the frame. Returns
    (frame_rows, agree_count). Consensus tolerances are looser than the
    match tolerances: anchors vote on the frame, they are not re-matched."""
    best = None
    best_score = -1
    for obj_rows, comp_rows in anchor_pairs:
        f = _frame_from_anchor(obj_rows, comp_rows)
        if f is None:
            continue
        score = sum(1 for o, c in anchor_pairs
                    if _frame_agrees(f, o, c, scene_scale, t_tol=1e-3, r_tol=5e-3))
        if score > best_score:
            best, best_score = f, score
        if score == len(anchor_pairs):
            break
    if best is None:
        return identity_frame(), 0
    return best, best_score


def describe_frame(frame) -> str:
    if frame is None:
        return "unknown"
    rot = [row[:3] for row in frame[:3]]
    if rot == _CANDIDATE_ROTS[0]:
        label = "aligned (Z-up)"
    elif rot == _CANDIDATE_ROTS[1]:
        label = "rotated +90° about X (Y-up import)"
    elif rot == _CANDIDATE_ROTS[2]:
        label = "rotated −90° about X (Y-up import)"
    elif rot == _CANDIDATE_ROTS[3]:
        label = "rotated 180° about X"
    else:
        label = "non-identity frame"
    if any(abs(frame[i][3]) > 1e-6 for i in range(3)):
        label += ", offset from origin"
    return label


def _component_dims(component: Component) -> Optional[List[float]]:
    if component.bbox_min is None or component.bbox_max is None:
        return None
    return sorted(abs(component.bbox_max[i] - component.bbox_min[i]) for i in range(3))


def _object_dims(obj, scene_scale: float) -> Optional[List[float]]:
    try:
        dims = [float(d) * scene_scale for d in obj.dimensions]
    except (AttributeError, TypeError):
        return None
    if max(dims) <= 0.0:
        return None
    return sorted(dims)


def _dims_agree(a: List[float], b: List[float]) -> bool:
    for x, y in zip(a, b):
        # Tessellation shifts bounding boxes, hence relative not absolute.
        if abs(x - y) > _BBOX_REL_TOL * max(x, y, 1e-9):
            return False
    return True


def collect_candidates(objects=None):
    """Scene objects that may carry manifest geometry. INSTANCES-mode
    prototypes (STEP_name without STEP_uuid, identity-placed in the hidden
    components collection) are excluded — the occurrence empties are the
    things that sit at the manifest transforms."""
    if objects is None:
        if bpy is None:
            return []
        objects = list(bpy.context.scene.objects)
    out = []
    for obj in objects:
        try:
            if obj.type == "ARMATURE":
                continue
            if "RIG_group_empty" in obj.keys() or "RIG_helper" in obj.keys():
                continue
        except (AttributeError, TypeError):
            continue
        key = get_step_key(obj)
        if key.name is not None and key.uuid is None:
            continue
        out.append(obj)
    return out


def _basename(path) -> str:
    if not path:
        return ""
    return str(path).replace("\\", "/").rsplit("/", 1)[-1].casefold()


def match(manifest: Manifest, objects=None) -> MatchReport:
    report = MatchReport()
    candidates = collect_candidates(objects)
    scene_scale = _scene_scale()

    # A long-lived test scene accumulates imports of OTHER step files; their
    # objects must never compete for this manifest's components. Objects from
    # the manifest's own file are identified by the STEP_file tag — but only
    # when at least one exists, so foreign importers (no tag at all) keep
    # their fuzzy-step chance in an otherwise empty scene.
    want_file = _basename(getattr(manifest, "step_file", None))
    if want_file:
        same_file = [o for o in candidates
                     if _basename(get_step_key(o).file) == want_file]
        if same_file and len(same_file) != len(candidates):
            print("[SWTB match] restricting to %d object(s) imported from %r "
                  "(%d other object(s) in the scene ignored)"
                  % (len(same_file), want_file, len(candidates) - len(same_file)))
            candidates = same_file

    group_of = {}
    for g in manifest.rigid_groups:
        for cid in g.components:
            group_of[cid] = g.id
    comps = {c.id: c for c in manifest.components
             if c.id in group_of and not c.suppressed}

    by_uuid = {}
    for obj in candidates:
        key = get_step_key(obj)
        if key.uuid is not None:
            by_uuid[(key.file, key.uuid)] = obj

    todo = dict(comps)
    pool = list(candidates)
    claimed = {}

    def claim(component_id, obj, step):
        obj["RIG_component_id"] = component_id
        obj["RIG_group"] = group_of[component_id]
        report.matched.append(MatchEntry(
            component_id=component_id,
            object_name=obj.name,
            step=step,
            confidence=_METHOD_NAMES[step],
        ))
        claimed[component_id] = obj
        todo.pop(component_id, None)
        pool.remove(obj)

    # Ambiguity is NOT terminal: identical occurrence paths (two instances of
    # the same subassembly rebuild the same product-name path) are routine,
    # and the transform step downstream tells the twins apart. A component is
    # reported ambiguous only when it reaches the end still unresolved.
    ambiguous_seen = {}

    def note_ambiguous(component_id, objs):
        ambiguous_seen[component_id] = [o.name for o in objs]

    # Pre-seed from a previous run; a tag pointing at an id this manifest
    # does not know is stale and ignored.
    for obj in list(pool):
        try:
            tagged = obj.get("RIG_component_id")
        except (AttributeError, TypeError):
            tagged = None
        if tagged in todo:
            claim(tagged, obj, 0)

    # Step 1: exact STEP_name plus rebuilt occurrence path.
    paths = {}
    for obj in pool:
        p = _occurrence_path(obj, by_uuid)
        if p is not None:
            paths[id(obj)] = _norm_path(p)
    for cid in sorted(todo):
        comp = todo[cid]
        if comp.step_occurrence_path is None:
            continue
        want = _norm_path(comp.step_occurrence_path)
        hits = [o for o in pool
                if get_step_key(o).name == comp.step_name
                and paths.get(id(o)) == want]
        if len(hits) == 1:
            claim(cid, hits[0], 1)
        elif len(hits) > 1:
            note_ambiguous(cid, hits)

    # Step 2: path equality with instance suffixes and separators normalised.
    for cid in sorted(todo):
        comp = todo[cid]
        if comp.step_occurrence_path is None:
            continue
        want = _loose_path(comp.step_occurrence_path)
        hits = [o for o in pool
                if id(o) in paths and _loose_path(paths[id(o)]) == want]
        if len(hits) == 1:
            claim(cid, hits[0], 2)
        elif len(hits) > 1:
            note_ambiguous(cid, hits)

    # The scene frame: estimated from the name/path anchors when there are
    # any. Without them (FLAT imports, foreign importers), uniquely-named
    # components still anchor a FULL frame — rotation and translation, so
    # an import dropped at the 3D cursor is found, not only an up-axis
    # swap. Only when every name is duplicated do the canonical up-axis
    # conversions compete on how many components each one names-and-places
    # (they carry no translation — transform agreement is the only signal
    # that can disambiguate identical names).
    unit_scale = 1.0 / scene_scale
    anchor_pairs = [(_matrix_rows(claimed[e.component_id]),
                     _component_rows_units(comps[e.component_id], unit_scale))
                    for e in report.matched]
    if anchor_pairs:
        frame, agree = estimate_frame(anchor_pairs, scene_scale)
    else:
        weak = []
        for cid in sorted(todo):
            comp = todo[cid]
            hits = [o for o in pool if get_step_key(o).name == comp.step_name]
            if len(hits) == 1:
                weak.append((_matrix_rows(hits[0]),
                             _component_rows_units(comp, unit_scale)))
        if weak:
            frame, agree = estimate_frame(weak, scene_scale)
        else:
            frame, agree = identity_frame(), 0
        if agree == 0:
            for rot in _CANDIDATE_ROTS:
                cand = _frame_from_rot(rot)
                score = 0
                for cid in todo:
                    comp = todo[cid]
                    crows = _component_rows_units(comp, unit_scale)
                    hits = [o for o in pool
                            if get_step_key(o).name == comp.step_name
                            and _frame_agrees(cand, _matrix_rows(o), crows, scene_scale)]
                    if len(hits) == 1:
                        score += 1
                if score > agree:
                    frame, agree = cand, score
    report.frame_rows = frame
    report.frame_agree = agree

    # Pre-seeded claims are trusted history, and history goes stale: a tag
    # written by an earlier manifest can sit on an object this manifest's
    # component no longer describes (rounds of re-import and re-export in one
    # scene). Now that a frame exists, any step-0 claim whose object is not
    # where the component says gets reverted and re-matched from scratch —
    # the frame vote is majority-based, so a minority of stale tags cannot
    # defend themselves by defining the frame.
    stale = [e for e in report.matched
             if e.step == 0 and not _frame_agrees(
                 frame, _matrix_rows(claimed[e.component_id]),
                 _component_rows_units(comps[e.component_id], unit_scale),
                 scene_scale, t_tol=1e-3, r_tol=5e-3)]
    for e in stale:
        obj = claimed.pop(e.component_id)
        report.matched.remove(e)
        todo[e.component_id] = comps[e.component_id]
        pool.append(obj)
        for tag in ("RIG_component_id", "RIG_group"):
            try:
                del obj[tag]
            except (KeyError, TypeError):
                pass
        print("[SWTB match] stale tag: %s no longer sits at %s's transform; "
              "re-matching both" % (obj.name, e.component_id))

    # Step 3: same STEP_name plus transform agreement under the scene frame.
    for cid in sorted(todo):
        comp = todo[cid]
        crows = _component_rows_units(comp, unit_scale)
        hits = [o for o in pool
                if get_step_key(o).name == comp.step_name
                and _frame_agrees(frame, _matrix_rows(o), crows, scene_scale)]
        if len(hits) == 1:
            claim(cid, hits[0], 3)
        elif len(hits) > 1:
            note_ambiguous(cid, hits)

    # Step 4: bounding-box dimensions tiebreaker.
    for cid in sorted(todo):
        comp = todo[cid]
        want = _component_dims(comp)
        if want is None:
            continue
        hits = []
        for o in pool:
            name = get_step_key(o).name
            if name is not None and name != comp.step_name:
                continue
            dims = _object_dims(o, scene_scale)
            if dims is not None and _dims_agree(dims, want):
                hits.append(o)
        if len(hits) == 1:
            claim(cid, hits[0], 4)
        elif len(hits) > 1:
            note_ambiguous(cid, hits)

    # Step 5: fuzzy label match, unique in both directions only.
    fuzzy_comps = {}
    for cid in sorted(todo):
        fuzzy_comps.setdefault(_fuzzy_label(todo[cid].step_name), []).append(cid)
    fuzzy_objs = {}
    for o in pool:
        label = get_step_key(o).name
        if label is None:
            label = o.name
        fuzzy_objs.setdefault(_fuzzy_label(label), []).append(o)
    for label, cids in sorted(fuzzy_comps.items()):
        objs = fuzzy_objs.get(label, [])
        if len(cids) == 1 and len(objs) == 1 and objs[0] in pool and cids[0] in todo:
            claim(cids[0], objs[0], 5)

    report.ambiguous = sorted(
        (cid, names) for cid, names in ambiguous_seen.items() if cid in todo)
    still_ambiguous = {cid for cid, _ in report.ambiguous}
    report.unmatched = sorted(cid for cid in todo if cid not in still_ambiguous)
    report.unclaimed_objects = [o.name for o in pool]

    # Say WHY, per failure — the console line is what turns the next
    # "unmatched: c004" report into a one-look diagnosis.
    for cid, names in report.ambiguous:
        print("[SWTB match] ambiguous: %s (%s) — candidates %s"
              % (cid, comps[cid].step_name, ", ".join(names)))
    for cid in report.unmatched:
        comp = comps[cid]
        crows = _component_rows_units(comp, unit_scale)
        pred = apply_frame(frame, crows)
        name_hits = [o for o in pool if get_step_key(o).name == comp.step_name]
        if not name_hits:
            print("[SWTB match] unmatched: %s — no unclaimed object carries "
                  "STEP_name %r" % (cid, comp.step_name))
            continue
        nearest = []
        for o in name_hits:
            rows = _matrix_rows(o)
            d = sum((rows[i][3] - pred[i][3]) ** 2 for i in range(3)) ** 0.5
            nearest.append((d * scene_scale, o.name))
        nearest.sort()
        detail = ", ".join("%s at %.4f m off" % (n, d) for d, n in nearest[:3])
        print("[SWTB match] unmatched: %s (%s) — same-name candidates rejected "
              "on transform: %s" % (cid, comp.step_name, detail))
    return report
