# NVIDIA 580 Driver Test

Status date: 2026-09-12 (Asia/Shanghai)

Current state: **580.173.02 ACTIVE; POST-REBOOT DRIVER GATE PASS; ISAAC RUNTIME GATES PENDING**

## Before

- Driver: NVIDIA 595.84, APT `nvidia-driver-595-open`
- GPU: GeForce RTX 5090, PCI `10de:2b85`
- Kernel: Ubuntu `6.8.0-136-generic`
- NVIDIA kernel module: open DKMS 595.84
- CUDA toolkit: 12.8.61, independently installed under `/usr/local/cuda-12.8`
- PyTorch: unavailable in `/usr/bin/python3`; no Conda command found
- Isaac 5: 595.84 RTX SceneDB native startup crash baseline; 580.173.02 historical GUI PASS
- Isaac 6: 595.84 current GUI PASS is user-supplied and was not relaunched; 580.173.02 historical GUI PASS is log-backed

Detailed evidence: [driver_580_preflight_report.md](driver_580_preflight_report.md) and [before/](before/).

## Preflight

- Exact candidate: `nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1`
- RTX 5090 support: VERIFIED from exact PCI ID, Ubuntu driver selection, and NVIDIA 580.173.02 supported-chips metadata
- APT simulation: resolved; 20 NVIDIA 595 packages removed, 20 NVIDIA 580 packages installed; no unrelated removals
- Exact 595.84 recovery package: AVAILABLE_ONLINE
- SSH daemon: enabled, active, listening
- External SSH login: UNVERIFIED
- Active NVIDIA compute process: none observed
- Active project processes: 24 recorder nodes observed
- Current risk: HIGH / NOT SWITCH-READY

## Driver transition

**PACKAGE AND DKMS INSTALL PASS; REBOOT PENDING.** After confirming physical-console recovery and accepting that recorder processes may terminate on reboot, the user locally authenticated and ran:

```bash
sudo apt-get install --allow-downgrades nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1
```

APT recorded a successful transaction from 13:44:36 to 13:45:07. The exact 580 meta-package and open DKMS package are installed, `dpkg --audit` is clean, and DKMS reports `nvidia/580.173.02` installed for `6.8.0-136-generic`. `modinfo` resolves the on-disk module as 580.173.02, and APT regenerated the current kernel's initramfs.

Before reboot, the already-loaded kernel module remains 595.84 while user-space NVML is 580.173. Therefore `nvidia-smi` currently reports `Driver/library version mismatch`. This is the expected mixed pre-reboot transition state, not a failed post-reboot result. No Isaac application may be launched in this state.

Evidence: [phase_b_post_install_pre_reboot_20260912_1345.md](logs/phase_b_post_install_pre_reboot_20260912_1345.md).

## After reboot

**PASS.** The machine rebooted at 2026-09-12 13:52 CST. At 13:59 CST, `nvidia-smi`, the loaded module, `modinfo`, DKMS, and installed packages all consistently reported 580.173.02 on kernel `6.8.0-136-generic`. `dpkg --audit` was clean.

Evidence: [post_reboot_gate1_20260912_1359.md](after_580/post_reboot_gate1_20260912_1359.md).

## NVIDIA status

**PASS.** RTX 5090 is visible and operational under loaded driver 580.173.02. No NVIDIA Xid was found in the current-boot kernel journal, and no GPU compute process was active at the check point.

## Vulkan/OpenGL

`vulkaninfo` and `glxinfo` were not installed during baseline capture. No dependency was installed.

## CUDA/PyTorch

**BASELINE CONSISTENT.** The existing CUDA Toolkit remains 12.8.61 at `/usr/local/cuda-12.8`; `nvidia-smi` advertises driver API compatibility up to CUDA 13.0. System Python remains without PyTorch, so no PyTorch CUDA allocation test was possible and no dependency was installed.

## Isaac5 GUI

**NOT RUN ON 580 IN THIS TEST.** Historical 580.173.02 GUI PASS remains historical evidence only.

## Isaac6 GUI

**NOT RUN ON 580 IN THIS TEST.** Historical 580.173.02 GUI PASS remains historical evidence only.

## Rollback readiness

- Package rollback: READY_ONLINE via `nvidia-driver-595-open=595.84-0ubuntu0.22.04.1`
- Recovery channel: SSH service ready; external login unverified
- Offline package cache: not prepared
- End-to-end rollback: CONDITIONALLY READY, not yet proven

## Final decision

**DRIVER TRANSITION PASS; PROCEED TO BOUNDED ISAAC 5 GUI GATE.**

580.173.02 is now the active loaded driver and post-reboot Gate 1 passed. This does not yet establish Isaac renderer compatibility. The next gate is a user-observed Isaac Sim 5.1 default/empty GUI launch: require a real visible 3D viewport, successful SceneDB initialization, at least 60 seconds without native crash, and clean user-initiated teardown before loading Mecanum730 or starting Isaac 6.
