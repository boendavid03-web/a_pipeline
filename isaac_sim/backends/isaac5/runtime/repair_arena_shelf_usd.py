#!/usr/bin/env python3
"""Create an isolated Arena shelf USD with valid physics and material scope.

The pinned Arena shelf asset has RigidBodyAPI on both /bookshelf and
/bookshelf/link. Isaac/PhysX rejects that hierarchy. Its materials live under
/Looks, outside the /bookshelf reference scope, so Isaac ignores bindings.
This utility repairs a copied USD and never edits the protected Arena source.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    # Isaac's USD Python bindings become available after SimulationApp starts.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "enable_livestream": False})
    try:
        from pxr import Sdf, Usd, UsdPhysics

        stage = Usd.Stage.Open(str(args.source))
        if stage is None:
            raise RuntimeError(f"cannot open USD: {args.source}")
        root = stage.GetPrimAtPath("/bookshelf")
        link = stage.GetPrimAtPath("/bookshelf/link")
        if not root.IsValid() or not link.IsValid():
            raise RuntimeError("expected /bookshelf and /bookshelf/link prims")
        if not root.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("source root has no RigidBodyAPI; refusing silent rewrite")
        if not link.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("source child has no RigidBodyAPI; refusing silent rewrite")
        looks = stage.GetPrimAtPath("/Looks")
        if not looks.IsValid():
            raise RuntimeError("source has no /Looks scope; refusing silent rewrite")

        root.RemoveAPI(UsdPhysics.RigidBodyAPI)
        layer = stage.GetRootLayer()
        old_scope = Sdf.Path("/Looks")
        new_scope = Sdf.Path("/bookshelf/Looks")
        if not Sdf.CopySpec(layer, old_scope, layer, new_scope):
            raise RuntimeError("failed to copy materials into /bookshelf")
        for prim in stage.Traverse():
            if not prim.GetPath().HasPrefix(root.GetPath()):
                continue
            for relation in prim.GetRelationships():
                targets = relation.GetTargets()
                if any(target.HasPrefix(old_scope) for target in targets):
                    relation.SetTargets(
                        [target.ReplacePrefix(old_scope, new_scope) for target in targets]
                    )
            for attribute in prim.GetAttributes():
                connections = attribute.GetConnections()
                if any(target.HasPrefix(old_scope) for target in connections):
                    attribute.SetConnections(
                        [target.ReplacePrefix(old_scope, new_scope) for target in connections]
                    )
        stage.RemovePrim(old_scope)
        stage.SetDefaultPrim(root)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not layer.Export(str(args.output)):
            raise RuntimeError(f"failed to export USD: {args.output}")

        check = Usd.Stage.Open(str(args.output))
        assert check is not None
        root_enabled = check.GetPrimAtPath("/bookshelf").HasAPI(UsdPhysics.RigidBodyAPI)
        link_enabled = check.GetPrimAtPath("/bookshelf/link").HasAPI(UsdPhysics.RigidBodyAPI)
        if root_enabled or not link_enabled:
            raise RuntimeError("exported shelf did not preserve the expected physics APIs")
        if not check.GetPrimAtPath(new_scope).IsValid():
            raise RuntimeError("exported shelf has no scoped materials")
        for prim in check.Traverse():
            if not prim.GetPath().HasPrefix(root.GetPath()):
                continue
            for relation in prim.GetRelationships():
                if any(target.HasPrefix(old_scope) for target in relation.GetTargets()):
                    raise RuntimeError(f"out-of-scope material target in {prim.GetPath()}")
            for attribute in prim.GetAttributes():
                if any(target.HasPrefix(old_scope) for target in attribute.GetConnections()):
                    raise RuntimeError(f"out-of-scope material connection in {prim.GetPath()}")
        print(
            f"root_rigidbody={root_enabled} "
            f"link_rigidbody={link_enabled} "
            f"materials={len(check.GetPrimAtPath(new_scope).GetChildren())} "
            f"output={args.output}",
            flush=True,
        )
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
