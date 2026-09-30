# NVIDIA 580 post-reboot Gate 1

Timestamp: 2026-09-12 13:59 CST

Result: **PASS**

## Runtime identity

- GPU: NVIDIA GeForce RTX 5090
- `nvidia-smi`: 580.173.02
- Loaded `/proc/driver/nvidia/version`: 580.173.02 open kernel module
- `modinfo -F version nvidia`: 580.173.02
- DKMS: `nvidia/580.173.02, 6.8.0-136-generic, x86_64: installed`
- Kernel: `6.8.0-136-generic`
- Packages: `nvidia-driver-580-open` and `nvidia-dkms-580-open`, both `580.173.02-0ubuntu0.22.04.1`
- `dpkg --audit`: no output
- Boot time: 2026-09-12 13:52:22 CST

The installed package, on-disk module, loaded module, DKMS registration, and NVML runtime now agree. The pre-reboot driver/library mismatch is resolved.

## Additional read-only checks

- Current-boot kernel journal: NVIDIA 580.173.02 loaded; no NVIDIA Xid found.
- The module-verification/taint line is not a module-load failure. Secure Boot was already verified disabled during preflight.
- Active GPU clients were graphical desktop processes; no compute process was reported.
- Existing CUDA Toolkit: `/usr/local/cuda-12.8/bin/nvcc`, CUDA 12.8.61.
- The CUDA 13.0 value shown by `nvidia-smi` is the driver's advertised API compatibility level, not the locally installed toolkit version.
- `/usr/bin/python3` remains Python 3.10.12 without PyTorch, matching the pre-switch baseline. No dependency was installed.

## Boundary

This gate proves the driver transition and basic GPU/desktop availability. It does not prove Vulkan rendering, Isaac Sim 5.1 SceneDB startup, a visible 3D viewport, or Isaac Sim 6.0.1 compatibility. Those require their separate runtime gates.
