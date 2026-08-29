#!/usr/bin/env python3
"""Prepare a process-local Vulkan Profiles workaround for Isaac Sim 5.1.

NVIDIA 595 can expose VkPhysicalDeviceMaintenance3Properties.
maxMemoryAllocationSize as UINT64_MAX.  Kit 107.3's RTX SceneDB crashes while
handling that sentinel.  This helper installs a checksum-verified Khronos
Profiles layer in the user's cache and generates a profile that reports a
finite allocation limit.  It does not modify the NVIDIA driver or system
Vulkan configuration.
"""

from __future__ import annotations

import argparse
from ctypes import (
    CDLL,
    POINTER,
    Structure,
    addressof,
    byref,
    c_char,
    c_char_p,
    c_int32,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from hashlib import sha256
from json import dump
from os import environ
from pathlib import Path


LAYER_SHA256 = "14e5b56e6006ed5fa75536549f87d8c4228d7640d28f4e14eacd460f647d3896"
PROFILE_NAME = "VP_RTX_driver_compat_generated"
NVIDIA_VENDOR_ID = 0x10DE
UINT64_MAX = (1 << 64) - 1
TWO_MIB = 2 * 1024 * 1024
ALLOCATION_CAP = 4 * 1024 * 1024 * 1024 - TWO_MIB
CACHE_ROOT = (
    Path(environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache")))
    / "vulkan-rtx-compat"
)
LOCAL_LAYER = (
    Path(__file__).resolve().parent.parent
    / "runtime"
    / "vulkan_595_compat"
    / "libVkLayer_khronos_profiles.so"
)


class VkApplicationInfo(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("pApplicationName", c_char_p),
        ("applicationVersion", c_uint32),
        ("pEngineName", c_char_p),
        ("engineVersion", c_uint32),
        ("apiVersion", c_uint32),
    ]


class VkInstanceCreateInfo(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("flags", c_uint32),
        ("pApplicationInfo", c_void_p),
        ("enabledLayerCount", c_uint32),
        ("ppEnabledLayerNames", c_void_p),
        ("enabledExtensionCount", c_uint32),
        ("ppEnabledExtensionNames", c_void_p),
    ]


class VkPhysicalDeviceProperties(Structure):
    _fields_ = [
        ("apiVersion", c_uint32),
        ("driverVersion", c_uint32),
        ("vendorID", c_uint32),
        ("deviceID", c_uint32),
        ("deviceType", c_uint32),
        ("deviceName", c_char * 256),
        ("pipelineCacheUUID", c_uint8 * 16),
        ("_rest", c_uint8 * 2048),
    ]


class VkPhysicalDeviceProperties2(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("properties", VkPhysicalDeviceProperties),
    ]


class VkPhysicalDeviceMaintenance3Properties(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("maxPerSetDescriptors", c_uint32),
        ("maxMemoryAllocationSize", c_uint64),
    ]


def file_hash(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def query_nvidia_limits() -> list[tuple[str, int]]:
    vk = CDLL("libvulkan.so.1")
    create_instance = vk.vkCreateInstance
    create_instance.argtypes = [POINTER(VkInstanceCreateInfo), c_void_p, POINTER(c_void_p)]
    create_instance.restype = c_int32
    enumerate_devices = vk.vkEnumeratePhysicalDevices
    enumerate_devices.argtypes = [c_void_p, POINTER(c_uint32), POINTER(c_void_p)]
    enumerate_devices.restype = c_int32
    get_properties = vk.vkGetPhysicalDeviceProperties2
    get_properties.argtypes = [c_void_p, POINTER(VkPhysicalDeviceProperties2)]
    destroy_instance = vk.vkDestroyInstance
    destroy_instance.argtypes = [c_void_p, c_void_p]

    app = VkApplicationInfo(
        sType=0,
        pApplicationName=b"isaacsim-rtx-595-compat",
        apiVersion=(1 << 22) | (1 << 12),
    )
    create_info = VkInstanceCreateInfo(
        sType=1,
        pApplicationInfo=c_void_p(addressof(app)),
    )
    instance = c_void_p()
    result = create_instance(byref(create_info), None, byref(instance))
    if result != 0:
        raise RuntimeError(f"vkCreateInstance failed with code {result}")

    devices_found: list[tuple[str, int]] = []
    try:
        count = c_uint32()
        result = enumerate_devices(instance, byref(count), None)
        if result != 0 or count.value == 0:
            raise RuntimeError(f"vkEnumeratePhysicalDevices failed with code {result}")
        devices = (c_void_p * count.value)()
        result = enumerate_devices(instance, byref(count), devices)
        if result != 0:
            raise RuntimeError(f"vkEnumeratePhysicalDevices failed with code {result}")

        for device in devices:
            maintenance3 = VkPhysicalDeviceMaintenance3Properties(sType=1000168000)
            properties = VkPhysicalDeviceProperties2(
                sType=1000059001,
                pNext=c_void_p(addressof(maintenance3)),
            )
            get_properties(device, byref(properties))
            if properties.properties.vendorID != NVIDIA_VENDOR_ID:
                continue
            name = bytes(properties.properties.deviceName).split(b"\0", 1)[0].decode(
                errors="replace"
            )
            devices_found.append((name, int(maintenance3.maxMemoryAllocationSize)))
    finally:
        destroy_instance(instance, None)

    if not devices_found:
        raise RuntimeError("no NVIDIA Vulkan device was found")
    return devices_found


def ensure_layer() -> Path:
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    if not LOCAL_LAYER.is_file():
        raise RuntimeError(f"locally built Vulkan Profiles layer is missing: {LOCAL_LAYER}")
    if file_hash(LOCAL_LAYER) != LAYER_SHA256:
        raise RuntimeError("locally built Vulkan Profiles layer failed SHA-256 verification")
    return LOCAL_LAYER


def write_profile(layer: Path) -> None:
    profile_dir = CACHE_ROOT / "profiles"
    manifest_dir = CACHE_ROOT / "xdg-data" / "vulkan" / "implicit_layer.d"
    profile_dir.mkdir(parents=True, exist_ok=True)
    manifest_dir.mkdir(parents=True, exist_ok=True)

    with (profile_dir / f"{PROFILE_NAME}.json").open("w", encoding="utf-8") as stream:
        dump(
            {
                "$schema": "https://schema.khronos.org/vulkan/profiles-0.8-latest.json#",
                "capabilities": {
                    "RTX_DRIVER_COMPAT": {
                        "properties": {
                            "VkPhysicalDeviceMaintenance3Properties": {
                                "maxMemoryAllocationSize": ALLOCATION_CAP
                            }
                        }
                    }
                },
                "profiles": {
                    PROFILE_NAME: {
                        "version": 1,
                        "api-version": "1.1.0",
                        "label": "Isaac Sim 5.1 RTX 595 compatibility",
                        "description": "Process-local finite Vulkan allocation limit",
                        "contributors": {"local": {"github": "https://github.com/yushijinhun"}},
                        "history": [
                            {
                                "revision": 1,
                                "date": "2026-05-10",
                                "author": "local",
                                "comment": "Generated compatibility profile",
                            }
                        ],
                        "capabilities": ["RTX_DRIVER_COMPAT"],
                    }
                },
            },
            stream,
            indent=2,
        )
        stream.write("\n")

    with (manifest_dir / "VkLayer_KHRONOS_profiles.json").open(
        "w", encoding="utf-8"
    ) as stream:
        dump(
            {
                "file_format_version": "1.2.1",
                "layer": {
                    "name": "VK_LAYER_KHRONOS_profiles",
                    "type": "GLOBAL",
                    "library_path": str(layer),
                    "api_version": "1.4.341",
                    "implementation_version": "1",
                    "description": "Khronos Profiles layer",
                    "enable_environment": {"RTX_VULKAN_COMPAT_ENABLE_LAYER": "1"},
                    "disable_environment": {"RTX_VULKAN_COMPAT_DISABLE_LAYER": ""},
                },
            },
            stream,
            indent=2,
        )
        stream.write("\n")


def print_limits(label: str) -> list[tuple[str, int]]:
    limits = query_nvidia_limits()
    for name, limit in limits:
        print(f"{label}: {name}: maxMemoryAllocationSize={limit}")
    return limits


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()

    limits = print_limits("Vulkan")
    unsafe = any(limit > ALLOCATION_CAP for _, limit in limits)
    if args.verify:
        if unsafe:
            print("ERROR: compatibility profile is not active")
            return 3
        print("Compatibility profile verification passed")
        return 0

    if not unsafe:
        print("Compatibility profile is not required for this driver")
        return 0
    if not any(limit == UINT64_MAX for _, limit in limits):
        print("WARNING: allocation limit is large but is not UINT64_MAX; applying finite cap")

    layer = ensure_layer()
    write_profile(layer)
    print(f"Prepared compatibility profile: {CACHE_ROOT}")
    print(f"Finite allocation cap: {ALLOCATION_CAP}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
