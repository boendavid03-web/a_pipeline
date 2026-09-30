"""Temporary, no-stage probe for Isaac 5 native navigation bindings."""

from isaacsim import SimulationApp


app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "fast_shutdown": True,
        "width": 1280,
        "height": 720,
    }
)
try:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.anim.navigation.bundle", True)
    for _ in range(8):
        app.update()
    import omni.anim.navigation.core as nav

    navigation = nav.acquire_interface()
    controller = navigation.create_controller()
    try:
        print("NAV_MODULE=" + str(nav.__file__), flush=True)
        print(
            "NAV_CONTROLLER_METHODS="
            + repr(sorted(name for name in dir(controller) if not name.startswith("_"))),
            flush=True,
        )
    finally:
        controller.destroy()
finally:
    app.close()
