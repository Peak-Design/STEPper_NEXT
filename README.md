# STEPper NEXT - STEP File Importer for Blender

Blender addon for importing STEP (`.step` / `.stp`), IGES (`.iges` / `.igs`) and BREP (`.brep` / `.brp`) files directly into Blender using the OpenCASCADE (OCC) geometry kernel. The produced mesh is a triangulation of the underlying CAD surface with smooth normals computed from the analytic shape geometry.

Originally created by **ambi** (Tommi Hyppanen). Now maintained by **Peak Design** (Oskaras Spalvys).

## Features

**Geometry**

- Direct STEP, IGES and BREP import via OpenCASCADE
- Analytic surface normals for smooth, seamless shading on curved surfaces
- Sharp edges marked from the CAD topology, with custom split normals
- Quality presets with unit-aware deflection — a physical 0.8 mm stays 0.8 mm whatever units the file uses — plus relative (adaptive) tessellation
- Robust handling of corrupted geometry: imports everything it can instead of skipping whole parts, with ShapeFix healing for damaged shapes
- Free edges and sketches optionally imported as curve objects

**Materials & UVs**

- Per-face vertex colors and automatic material creation from STEP color data
- Engineering material metadata (AP242/AP214 name, description, density) imported as custom properties, and optionally as named Blender materials
- Material database system for automatic material replacement on import
- UV generation from CAD surfaces, angle-based unwrap, or box projection — with optional real-world UV scale and automatic seams on cylindrical/closed faces

**Workflow**

- Non-blocking background import: Blender stays responsive, Esc cancels
- Viewport drag & drop (single or multiple files) and recursive folder batch import
- Pre-import analyzer with per-machine import-time estimates
- Regenerate parts at a different quality, Prune/Restore hierarchy, mesh cleanup
- Import options remembered between Blender sessions
- Part hierarchy preserved as flat collection, nested collections, parented empties, or collection instances
- Native C++ mesh extraction with multithreaded normal computation — up to 10x faster than v1.x

## Material Database

The material database lets you define mappings from generic STEP material names (e.g., "GRAY", "BLACK") to authored Blender materials. Once configured, materials are automatically replaced every time you import a STEP file.

![Material Mappings Panel](docs/material_mappings.png)

### Setup

1. Import a STEP file normally. Objects load with generic STEP materials.
2. Assign the Blender materials you want to each part (e.g., replace "GRAY" with "Stainless Steel" etc.).
3. In the **STEPper NEXT: Material DB** sidebar panel, click **New** to create a database. The addon scans the scene and records what each original STEP material was replaced with.
4. Manually assign/tweak material mappings in the mapping table if required.
5. The database is saved as a `.blend` file in the addon's `MaterialDB/` folder.

### Importing with a Database

Select a database from the dropdown in the STEP import dialog under **Material DB**. The selected database persists between sessions. When importing, all matching STEP materials are automatically replaced.

### Panel Buttons

| Button | Description |
|--------|-------------|
| **New** | Create a new database from the current scene. Scans all STEP objects and records current material assignments. If the same original material was replaced with different materials on different parts, the most common replacement wins. |
| **Duplicate** | Copy the active database under a new name. Useful for minor variations between projects. |
| **Load** | Reload mappings from the active database file and append its materials into the current file. |
| **Delete** (trash icon) | Delete the active database file. |
| **Update** | Scan the scene for any new original STEP material names not already in the database and add them. **Does not modify existing mappings.** USe this to expand and grow your material database. Does not auto-save. |
| **Save** | Write the current mappings and materials to the database file. |
| **Apply** | Apply the active database mappings to objects in the scene. Works with the **Selection only** checkbox to limit to selected objects. |

### Material Mappings Table

Each row shows an original STEP material name and a dropdown to pick the replacement Blender material. You can change any mapping and click **Save** to update the database.

### Notes

- Databases are stored in the `MaterialDB/` folder inside the addon directory.
- The active database selection is stored in addon preferences and persists across sessions and files.
- Original STEP material names are stored on each imported object as a `STEP_materials` custom property, so re-applying a different database always works correctly.
- Linked materials (e.g., from the Blender asset browser) are fully supported. A local copy is saved into the database file so it can be loaded in any `.blend` file.

## Engineering Materials (AP242 / AP214)

STEP files can carry the engineering material assigned in the source CAD system — name, description and density — alongside the geometry. Every import stores whatever it finds on each object as `STEP_material`, `STEP_material_desc` and `STEP_material_density` custom properties.

The **Engineering Materials** import option (on by default) additionally gives each part a single Blender material named after the CAD material, e.g. `AISI 304 Steel`, instead of the color-derived ones. That pairs naturally with the Material Database above: map `AISI 304 Steel` to your authored steel shader once, and every future import picks it up.

> **Not every CAD system exports this.** SOLIDWORKS does not write engineering material data into STEP at all, in any schema — and its "Export Appearances" option only carries flat per-face colours, since STEP has no representation for textures or PBR properties. CATIA and NX do export material data with the appropriate options enabled. Files without material data simply import without the custom properties.

## UV Maps

A single `UVMap` layer is created; the **UV Map** import option chooses its content.

| Mode | Description |
|------|-------------|
| **CAD Surface** | One island per CAD face, taken from the parametric surface coordinates. Fast, and the default. |
| **Unwrap** | Blender's angle-based unwrap with packed islands; sharp CAD edges act as seams. Slower on large assemblies. |
| **Box Project** | Triplanar projection with a world-unit tile size. |
| **None** | No UV layer. |

**Normalize UVs** (off by default) fits the UVs to the 0-1 square. With it off, UVs are scaled to real-world scene units instead — islands stay packed and are uniformly rescaled — so one shared material shows its texture at the same physical scale on every part, which is usually what you want for CAD.

**Split Closed Faces** (on by default) marks a UV seam along the parametric closure of cylinders, cones and tori, and across smooth-joined face groups that form closed tubes or rings. CAD data has no seam there, and unwrapping without one produces badly distorted islands. Shading is unaffected.

## Import Defaults

With **Remember import settings** enabled in the addon preferences (the default), the import dialog's options are saved after every import and restored in your next Blender session, so you configure them once. They are written out with Blender's normal preferences save, so keep *Save Preferences on Quit* on (Blender's default) or save preferences manually. Turn the option off to fall back to the fixed defaults in the preferences instead.

## Platform Support

| Platform | Status |
|----------|--------|
| **Windows 10+ (64-bit)** | Tested and supported |
| **macOS Apple Silicon (M1/M2/M3/M4)** | Experimental (untested) |
| **Linux (64-bit)** | Experimental (untested) |

## Requirements

- **Blender 5.1** with Python 3.13
- **Windows only:** Visual Studio C++ Redistributable: https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170 (vc_redist.x64.exe)

## Installation

STEPper NEXT ships as a Blender **extension** (since v2.3.0). The OpenCASCADE
(OCP) bindings are bundled as a wheel that Blender installs automatically.

1. Download the `.zip` for your platform from the [Releases](../../releases) page.
2. Drag & drop the `.zip` into a Blender window (or use **Edit > Preferences > Get Extensions >** drop-down menu **> Install from Disk...**).
3. Enable it under **Add-ons** if it isn't enabled automatically.

The importer panel will appear in **3D View > Tools panel > STEPper NEXT**.

> **Upgrading from v2.2.x or older (legacy addon):** remove the old
> "STEPper NEXT" entry from **Preferences > Add-ons** and restart Blender
> before installing the extension.

> **Note:** macOS and Linux builds are automatically compiled via GitHub Actions but have not been tested yet. If you encounter issues on these platforms, please [open an issue](../../issues).

## Uninstall / Update

Remove the extension from **Preferences > Get Extensions > Installed** (or Add-ons). To update, install the new `.zip` — Blender replaces the older version.

## Version History

| Version | Blender | Changes |
|---------|---------|---------|
| 2.4.5   | 5.1     | Engineering material import (AP242/AP214 name, description, density) as custom properties and optional named materials; single `UVMap` layer with a UV mode dropdown, real-world UV scaling and automatic seams on closed/cylindrical faces; import options remembered between sessions; parented-empties imports now leave everything at scale 1; recursive folder batch import; multi-file drag & drop fix |
| 2.4.4   | 5.1     | Code-review hardening: multi-level Prune/Restore, detail slider no longer overridden by the quality preset, batch import honours preference defaults, Regenerate/Reload fixed for IGES/BREP, background worker inherits preferences and cannot hang or outlive Blender, long CAD material names, edit-mode guards, tooltip and layout polish |
| 2.4.3   | 5.1     | Non-blocking background import (worker process, live progress, Esc to cancel) with a size threshold; identical output to direct import |
| 2.4.2   | 5.1     | IGES + BREP import, sketch/construction curves import, relative (adaptive) tessellation, pre-import analyzer with import-time estimates |
| 2.4.1   | 5.1     | SurfaceUV + BoxUV layers, viewport drag & drop, folder batch import, state-preserving Regenerate, Prune/Restore hierarchy tools, mesh cleanup |
| 2.4.0   | 5.1     | Quality presets with unit-aware deflection (physical mm in any unit system), collection-instances hierarchy mode, construction-geometry filters, per-instance color overrides, modern import dialog |
| 2.3.0   | 5.1     | Migrated OpenCASCADE bindings from pythonocc-core to OCP (cadquery-ocp-novtk 7.9.3.1.1); converted to Blender extension format with per-platform OCP wheels; added macOS Intel support; native mesh extraction reworked to a serialize handoff |
| 2.2.0   | 5.1     | Material database system for automatic material replacement, fixed apply-scale on instanced/multi-user meshes |
| 2.1.3   | 5.1     | Renamed to STEPper NEXT, auto-apply scale, skip empty objects, preferences now persist across sessions |
| 2.1.x   | 5.1     | Multithreaded normal computation, performance optimizations, crash fixes for corrupt STEP files |
| 2.1.0   | 5.1     | Updated to pythonocc-core 7.9.3 / Python 3.13, native C++ mesh extraction |
| 2.0.0   | 5.0     | Ported to Blender 5.0 API, added import diagnostics and failed parts reporting |
| 1.1.8   | 4.2.1   | Last release by ambi |

## Support

This addon is free and open source under the GPL v3 license.

**ambi** - original creator:
https://ambient.gumroad.com/l/stepper

**Peak Design** - current maintainer, tips welcome:
https://ko-fi.com/oskarasspalvys

## For Developers

The OpenCASCADE (OCP) bindings come from the [cadquery-ocp-novtk](https://pypi.org/project/cadquery-ocp-novtk/) wheels committed in `wheels/` (one per platform, referenced by `blender_manifest.toml` — Blender installs the matching one at extension install time). A small native mesh-extraction module (`native/`) ships per platform with its own plain-named OCCT subset in `native_libs/`; it talks to the importer via a BinTools serialize handoff, so it is independent of the Python bindings. Per-platform extension zips are built by GitHub Actions (`.github/workflows/release.yml`), which narrows the manifest to one platform/wheel per zip via `ci/make_platform_manifest.py` and runs `ci/smoke_test.py` before packaging.

For development in `scripts/addons` (legacy addon path, still supported via the retained `bl_info`): extract the Windows wheel into the addon folder so `import OCP` resolves — `python -m zipfile -e wheels/cadquery_ocp_novtk-*-win_amd64.whl .` (the extracted `OCP/` + `cadquery_ocp_novtk.libs/` folders are gitignored).

Parity testing: `blender -b --factory-startup --python ci/parity_harness.py -- <file.step> <out.json> ['<operator_kwargs_json>']` dumps a deterministic scene snapshot (objects, mesh counts, materials, transforms, collections) for diffing. Reference snapshots live in `ci/baselines/`, together with the self-contained `mat_ap242.step` / `mat_ap214.step` fixtures used to test engineering-material import; the other baselines reference local corpus files by absolute path and are meant as a change detector rather than a portable test suite.

Note: on Windows the addon/extension must not live under a path longer than ~250 characters, or the bundled OpenCASCADE DLLs will fail to load (default Blender paths are fine).

## License

This program is free software under the [GNU General Public License v3](https://www.gnu.org/licenses/gpl-3.0.html).

Copyright 2021 Tommi Hyppanen
Modified 2026 by Peak-Design
