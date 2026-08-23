# SPDX-License-Identifier: GPL-3.0-or-later
"""Matching hygiene tests, born from the live corpus 07 round three
(2026-08-22): a long-lived test scene — stale RIG_* tags from earlier
manifests, imports of several STEP files side by side, twin subassembly
instances sharing one occurrence path — reported "unmatched: c004" for a
component whose object sat exactly where the manifest said. Each test here
pins one of the hygiene rules that make the cascade survive such scenes.

No bpy — matching.py degrades to plain Python and the fake objects provide
the tiny surface it reads."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from STEPper_NEXT.rig import manifest, matching  # noqa: E402


def identity4():
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def translated(x, y, z):
    m = identity4()
    m[0][3], m[1][3], m[2][3] = x, y, z
    return m


class FakeObj:
    """The exact object surface matching.py touches, nothing more."""

    def __init__(self, name, props, matrix):
        self.name = name
        self.type = "MESH"
        self._props = dict(props)
        self.matrix_world = matrix

    def get(self, key, default=None):
        return self._props.get(key, default)

    def keys(self):
        return self._props.keys()

    def __setitem__(self, key, value):
        self._props[key] = value

    def __delitem__(self, key):
        del self._props[key]


def make_manifest(comps, step_file="asm.step"):
    """comps: list of (cid, step_name, occurrence_path, transform)."""
    return manifest.parse({
        "manifest_version": "1.0.0",
        "generator": {"name": "test", "version": "0"},
        "units": {"length": "meter", "angle": "radian"},
        "frame": {"handedness": "right", "up_axis": "Z",
                  "transform_convention": "row_major_4x4_global"},
        "step_export": {"file": step_file},
        "components": [
            {
                "id": cid,
                "sw_path": "asm/" + name + "-1",
                "step_name": name,
                "step_occurrence_path": path,
                "transform": transform,
            }
            for cid, name, path, transform in comps
        ],
        "rigid_groups": [
            {"id": "g%03d" % i, "name": name, "components": [cid], "grounded": i == 0}
            for i, (cid, name, _, _) in enumerate(comps)
        ],
        "joints": [],
        "loops": [],
        "warnings": [],
    })


class TwinPathsTest(unittest.TestCase):
    """Two instances of one subassembly rebuild the SAME product-name
    occurrence path — the exact-path step sees two hits for each twin
    component. That ambiguity must fall through to the transform step, not
    kill the component (the old cascade dropped it on the spot)."""

    def test_twin_paths_disambiguate_by_transform(self):
        m = make_manifest([
            ("c001", "leaf", "asm/leaf", translated(0.05, 0.02, 0.028)),
            ("c002", "leaf", "asm/leaf", translated(0.15, 0.02, 0.028)),
        ])
        root = FakeObj("asm", {"STEP_name": "asm", "STEP_uuid": 10,
                               "STEP_file": "asm.step"}, identity4())
        objs = [
            root,
            FakeObj("leaf", {"STEP_name": "leaf", "STEP_uuid": 1,
                             "STEP_parent": 10, "STEP_file": "asm.step"},
                    translated(0.05, 0.02, 0.028)),
            FakeObj("leaf.001", {"STEP_name": "leaf", "STEP_uuid": 2,
                                 "STEP_parent": 10, "STEP_file": "asm.step"},
                    translated(0.15, 0.02, 0.028)),
        ]
        report = matching.match(m, objects=objs)
        got = {e.component_id: e.object_name for e in report.matched}
        self.assertEqual({"c001": "leaf", "c002": "leaf.001"}, got)
        self.assertEqual([], report.ambiguous)
        self.assertEqual([], report.unmatched)


class StalePreseedTest(unittest.TestCase):
    """A RIG_component_id tag left by an earlier manifest can point at an
    object the current component no longer describes. With two healthy
    anchors outvoting it on the frame, the stale claim is reverted and the
    component re-matches to the object that actually sits at its
    transform."""

    def test_stale_preseed_reverts_to_transform_match(self):
        m = make_manifest([
            ("c001", "base", None, identity4()),
            ("c002", "arm", None, translated(0.1, 0.0, 0.0)),
            ("c003", "lid", None, translated(0.0, 0.1, 0.0)),
        ])
        stale = FakeObj("base.stale",
                        {"STEP_name": "base", "STEP_uuid": 1,
                         "STEP_file": "asm.step", "RIG_component_id": "c001",
                         "RIG_group": "g000"},
                        translated(0.5, 0.5, 0.0))
        fresh = FakeObj("base",
                        {"STEP_name": "base", "STEP_uuid": 2,
                         "STEP_file": "asm.step"},
                        identity4())
        arm = FakeObj("arm",
                      {"STEP_name": "arm", "STEP_uuid": 3,
                       "STEP_file": "asm.step", "RIG_component_id": "c002",
                       "RIG_group": "g001"},
                      translated(0.1, 0.0, 0.0))
        lid = FakeObj("lid",
                      {"STEP_name": "lid", "STEP_uuid": 4,
                       "STEP_file": "asm.step", "RIG_component_id": "c003",
                       "RIG_group": "g002"},
                      translated(0.0, 0.1, 0.0))
        report = matching.match(m, objects=[stale, fresh, arm, lid])
        got = {e.component_id: e.object_name for e in report.matched}
        self.assertEqual("base", got["c001"])
        self.assertEqual("arm", got["c002"])
        self.assertEqual("lid", got["c003"])
        self.assertIn("base.stale", report.unclaimed_objects)
        self.assertIsNone(stale.get("RIG_component_id"))
        self.assertIsNone(stale.get("RIG_group"))


class ForeignFileTest(unittest.TestCase):
    """A scene holding imports of several STEP files: objects from OTHER
    files must not compete, even when they sit at the same transforms (two
    exports of the same assembly do exactly that)."""

    def test_other_files_objects_are_ignored(self):
        m = make_manifest([
            ("c001", "base", None, identity4()),
        ], step_file="asm.step")
        mine = FakeObj("base",
                       {"STEP_name": "base", "STEP_uuid": 1,
                        "STEP_file": "C:/exports/asm.step"},
                       identity4())
        other = FakeObj("base.001",
                        {"STEP_name": "base", "STEP_uuid": 1,
                         "STEP_file": "C:/exports/other.step"},
                        identity4())
        report = matching.match(m, objects=[mine, other])
        got = {e.component_id: e.object_name for e in report.matched}
        self.assertEqual({"c001": "base"}, got)
        self.assertEqual([], report.ambiguous)
        self.assertEqual([], report.unmatched)


if __name__ == "__main__":
    unittest.main()
