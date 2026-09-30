#!/usr/bin/env python3
"""Build a separate hospital USD variant using normal maps in pinned Arena 5 source."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / "isaac_sim/arena_ws/src/arena/simulation-setup/gazebo_models"
HOSPITAL = ROOT / "isaac_sim/backends/isaac5/generated/arena_worlds/hospital"
EXPECTED = {
    "BedTable_meshes_BedTable_obj.usd": (
        "/World/Looks/BedTable/BedTable",
        "./textures/BedTable_Normal_Normal_Bump.tga",
        SOURCE / "BedTable/meshes/BedTable_Normal.png",
        "79cb1a78729bdbc48f5069091f611917695ccde97dd7bdb0d12ae4b9f2300cd7",
    ),
    "BPCart_meshes_Cart_BP_obj.usd": (
        "/World/Looks/BP/BP",
        "./textures/BP_Normal_Normal_Bump.tga",
        SOURCE / "BPCart/meshes/BP_BP_LP_Normal.png",
        "d13d5598fc148909e36b9c03f75191594c9469a0baf4b4207371626e30865658",
    ),
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "Wireframe", "multi_gpu": False})
from pxr import Sdf, Usd, UsdUtils  # noqa: E402


def main() -> int:
    output = HOSPITAL / "hospital_full_scene_source_png.usda"
    manifest = {"original_world": str(HOSPITAL / "hospital_full_scene.usda"),
                "derived_world": str(output), "source_commit": "3f142b25d88ce962c803b57cf20f38985d376dea",
                "semantic_equivalence_to_missing_tga": "unknown", "replacements": []}
    try:
        original = HOSPITAL / "hospital_full_scene.usda"
        text = original.read_text()
        for original_name, (prim_path, old_asset, source_png, expected_hash) in EXPECTED.items():
            if sha256(source_png) != expected_hash:
                raise RuntimeError(f"Pinned source PNG changed: {source_png}")
            old_usd = HOSPITAL / "meshes" / original_name
            new_name = old_usd.stem + "_source_png.usd"
            new_usd = old_usd.with_name(new_name)
            derived_png = HOSPITAL / "meshes" / "textures" / f"arena_source_{source_png.name}"
            if text.count(original_name) < 1:
                raise RuntimeError(f"World does not reference {original_name}")
            shutil.copy2(old_usd, new_usd)
            stage = Usd.Stage.Open(str(new_usd))
            prim = stage.GetPrimAtPath(prim_path)
            if not prim:
                raise RuntimeError(f"Missing material prim: {prim_path}")
            attr = prim.GetAttribute("inputs:normalmap_texture")
            value = attr.Get()
            if not isinstance(value, Sdf.AssetPath) or value.path != old_asset:
                raise RuntimeError(f"Unexpected authored normal map: {value}")
            derived_png.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_png, derived_png)
            attr.Set(Sdf.AssetPath(f"./textures/{derived_png.name}"))
            stage.GetRootLayer().Save()
            text = text.replace(original_name, new_name)
            manifest["replacements"].append({"mesh_usd_original": str(old_usd),
                                             "mesh_usd_derived": str(new_usd),
                                             "missing_mtl_asset": old_asset,
                                             "source_png": str(source_png),
                                             "source_png_sha256": expected_hash,
                                             "derived_png": str(derived_png),
                                             "derived_png_sha256": sha256(derived_png),
                                             "world_reference_count": text.count(new_name)})
        output.write_text(text)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(output))
        kit_mdl = {"OmniPBR.mdl", "OmniPBR_Opacity.mdl"}
        missing = [str(item) for item in unresolved if str(item) not in kit_mdl]
        manifest["dependencies"] = {"layers": len(layers), "files": len(assets), "missing": missing}
        manifest["derived_world_sha256"] = sha256(output)
        manifest["status"] = "PASS" if not missing else "FAIL"
        (HOSPITAL / "hospital_source_png_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print("ARENA_HOSPITAL_SOURCE_NORMALS=" + json.dumps(manifest), flush=True)
        return 0 if not missing else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
