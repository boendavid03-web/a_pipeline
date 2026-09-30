# Phase B post-install, pre-reboot checkpoint

Timestamp: 2026-09-12 13:45 CST

State: **580.173.02 packages and DKMS module installed; reboot required; 580 runtime not yet tested.**

## Package transaction

- APT command recorded in `/var/log/apt/history.log`:
  `apt-get install --allow-downgrades nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1`
- APT start: 2026-09-12 13:44:36
- APT end: 2026-09-12 13:45:07
- `dpkg --audit`: no output
- No `apt-get`, `dpkg`, DKMS build, or initramfs update process remained after completion.
- Installed meta-package: `nvidia-driver-580-open 580.173.02-0ubuntu0.22.04.1`
- Installed DKMS package: `nvidia-dkms-580-open 580.173.02-0ubuntu0.22.04.1`
- Exact rollback candidate remained available: `nvidia-driver-595-open 595.84-0ubuntu0.22.04.1`

## Kernel transition state

- Running kernel: `6.8.0-136-generic`
- DKMS: `nvidia/580.173.02, 6.8.0-136-generic, x86_64: installed`
- On-disk `modinfo -F version nvidia`: `580.173.02`
- Still-loaded module from the current boot: `595.84`
- User-space NVML library: `580.173`
- Consequently, pre-reboot `nvidia-smi` reported `Driver/library version mismatch`. This is expected during this mixed pre-reboot state and is not a post-reboot Gate 1 result.
- APT successfully generated `/boot/initrd.img-6.8.0-136-generic`.

## Boundary

Do not launch Isaac Sim or claim that driver 580 is active before reboot. The next operation is a normal system reboot, followed by the post-reboot `nvidia-smi`, `modinfo`, and `dkms status` gate.
