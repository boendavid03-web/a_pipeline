# Isaac 5.1 native navigation controller bridge

This is a narrow CPython 3.11 binding to the `INavController` API already
shipped with the local Isaac Sim 5.1 installation. It does not vendor a crowd
solver and does not modify the Isaac installation.

The bridge exposes agent creation, goal/speed updates, joint simulation,
island-aware closest-point queries, path inspection, result readback, and
teardown. The opt-in `--hunav-executor native_controller` experiment keeps
HuNav as the joint social-intent source while making this persistent native
controller the sole pedestrian motion owner. Dynamic robot obstacle
registration remains a separate gate.

Build with:

```bash
isaac_sim/backends/isaac5/native_controller_bridge/build.sh
```

The resulting module is loadable only in Isaac Sim 5.1's embedded CPython
3.11 process after `omni.anim.navigation.core` is enabled and a NavMesh has
been baked.
