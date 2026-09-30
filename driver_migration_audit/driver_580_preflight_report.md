# NVIDIA 580 Migration Preflight

调查时间：2026-09-12（Asia/Shanghai）

范围：仅完成 PHASE A 的只读系统审计、状态留档、APT dry-run 和回滚设计。未安装、卸载、hold、purge 或 autoremove 任何软件包；未停止 display manager；未重启；未启动 Isaac GUI；未结束任何现有进程。

## Preflight decision

**PRE-FLIGHT COMPLETE — PHASE B NOT AUTHORIZED**

- Current driver: `595.84`
- Candidate: `nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1`
- Current installation method: Ubuntu APT packages + open-kernel DKMS
- Candidate GPU support: VERIFIED for RTX 5090 PCI device `10de:2b85`
- Exact 595 recovery package: AVAILABLE from enabled Ubuntu repositories
- Package rollback: READY while the Ubuntu repositories and network remain available
- Recovery channel: SSH service is enabled, active, and listening; an external login was not tested, so end-to-end recovery is UNVERIFIED
- Current risk: **HIGH / NOT SWITCH-READY**
- Conditional risk after the blocking operational gates are cleared: **MEDIUM**

Current HIGH status is caused by the combination of an unverified external recovery login, active graphical remote-control sessions, 24 long-running project recorder processes that a reboot would terminate, a historical NVIDIA `.run` installation residue, and recurring current-boot NVRM reference-state failures. These findings do not invalidate the 580 hypothesis, but they prevent an unattended driver switch.

## Current system

- OS: Ubuntu 22.04.5 LTS
- Kernel: `6.8.0-136-generic`
- Architecture: x86_64
- Session: X11, `DISPLAY=:1`
- Display manager: GDM, active since 2026-09-06
- Root filesystem and `/boot`: same filesystem, approximately 1.1 TiB available, 39% used

Evidence: [system.txt](before/system.txt), [display_session.txt](before/display_session.txt), [display_manager.txt](before/display_manager.txt), [disk_space.txt](before/disk_space.txt).

## Current driver installation method

The active 595.84 driver is **APT-managed**, using the open kernel module and DKMS:

- `nvidia-driver-595-open 595.84-0ubuntu0.22.04.1`
- `nvidia-dkms-595-open 595.84-0ubuntu0.22.04.1`
- `dkms status`: `nvidia/595.84, 6.8.0-136-generic, x86_64: installed`
- `nvidia-smi`, `libcuda.so.595.84`, `libGLX_nvidia.so.595.84`, `libnvidia-glcore.so.595.84`, and the Xorg NVIDIA module are owned by their corresponding dpkg packages.
- The loaded module version is `595.84` and comes from `/lib/modules/6.8.0-136-generic/updates/dkms/`.
- APT history records the latest branch switch on 2026-08-10 as `apt install nvidia-driver-595-open`.

The generated DKMS `.ko` files are not directly listed as dpkg-owned files; this is normal for DKMS output and is not evidence of a `.run` module.

Evidence: [dpkg_nvidia_cuda.txt](before/dpkg_nvidia_cuda.txt), [dkms_status.txt](before/dkms_status.txt), [current_driver_file_provenance.txt](before/current_driver_file_provenance.txt), [apt_history_blocks_nvidia_cuda.txt](before/apt_history_blocks_nvidia_cuda.txt).

## Historical `.run` residue

The machine previously ran NVIDIA's `.run` installer for driver `570.211.01` on 2026-05-11. The installer log says that installation completed and `nvidia-xconfig` created `/etc/X11/xorg.conf`.

Current checks found:

- `/var/log/nvidia-installer.log` remains;
- `/etc/X11/xorg.conf` remains and identifies `nvidia-xconfig 570.211.01`;
- `/usr/bin/nvidia-uninstall`, `/usr/lib/nvidia/uninstall`, and `/usr/local/bin/nvidia-uninstall` are absent;
- the sampled active 595 userspace binaries/libraries are dpkg-owned;
- the active kernel module is the 595.84 DKMS build.

Therefore, this audit does **not** prove an active APT/`.run` binary mix. It does prove historical residue and a persistent `.run`-generated Xorg configuration. The configuration has been captured for recovery analysis and must not be silently deleted or regenerated during the 580 test.

Evidence: [historical_run_installer_and_xorg.txt](before/historical_run_installer_and_xorg.txt), [run_installer_indicators.txt](before/run_installer_indicators.txt).

## GPU

- GPU: NVIDIA GeForce RTX 5090
- PCI device: `10de:2b85`
- Current driver: `595.84`
- Current CUDA driver API level reported by `nvidia-smi`: 13.2
- Display active: yes
- `nvidia-smi` completed successfully.

Evidence: [nvidia_smi.txt](before/nvidia_smi.txt), [nvidia_smi_q.txt](before/nvidia_smi_q.txt), [pci_devices.txt](before/pci_devices.txt).

## Kernel

- Current kernel headers are installed.
- `/lib/modules/6.8.0-136-generic/build` resolves to `/usr/src/linux-headers-6.8.0-136-generic`.
- DKMS has a successful installed record for NVIDIA 595.84 on the current kernel.

Evidence: [kernel_headers.txt](before/kernel_headers.txt), [dkms_status.txt](before/dkms_status.txt).

## Secure Boot

- Secure Boot: disabled.
- The current module nevertheless contains a PKCS#7 signature made with the local Secure Boot module signing key.
- No Secure Boot setting or MOK enrollment was changed.

Evidence: [secure_boot.txt](before/secure_boot.txt).

## Display manager

- X11 session on `:1`.
- GDM is active.
- Xorg and GNOME Shell use the NVIDIA GPU.
- Awesun and ToDesk services are running; Awesun also has an active graphics process on the GPU.

A failed driver/desktop start can therefore remove the current graphical recovery path.

Evidence: [display_session.txt](before/display_session.txt), [display_manager.txt](before/display_manager.txt), [gpu_processes.txt](before/gpu_processes.txt).

## DKMS

`nvidia/595.84` is installed for `6.8.0-136-generic`. The candidate `nvidia-driver-580-open` depends on `nvidia-dkms-580-open` and exact-branch kernel source/common packages, so the proposed transition remains on the same APT + open-DKMS model used by the active driver.

## NVIDIA packages

The active userspace stack is the 595.84 Ubuntu package family. A 580 firmware package and removed-but-config-residue (`rc`) 580 package records remain from earlier APT branch switches; these are not active 580 userspace libraries. The audit did not purge them.

`dpkg --audit` produced no findings. An unprivileged `apt-get check` could not acquire the root-owned frontend lock, so it is not counted as a PASS. Both targeted APT simulations resolved successfully.

Evidence: [dpkg_nvidia_cuda.txt](before/dpkg_nvidia_cuda.txt), [package_health.txt](before/package_health.txt).

## Candidate 580 package

Enabled Ubuntu Jammy repositories offer both proprietary and open 580 branches. To preserve the current kernel-module model and reproduce the historical successful switches, the candidate is:

```text
nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1
```

Sources reported by APT:

- `jammy-updates/restricted`
- `jammy-security/restricted`

The APT dry-run proposes exactly 20 new 580 packages and removal of exactly 20 conflicting 595 packages. It does not propose removing Ubuntu desktop, GDM, Xorg, ROS, CUDA toolkit, Isaac files, or unrelated packages. It reports `0 upgraded, 20 newly installed, 20 to remove, 120 not upgraded`.

Evidence: [apt_candidate_580.txt](before/apt_candidate_580.txt), [apt_simulate_switch_580.txt](before/apt_simulate_switch_580.txt), [apt_sources_full_redacted.txt](before/apt_sources_full_redacted.txt).

## Exact available 580 version

Historical successful version and currently available APT version are the same:

```text
580.173.02-0ubuntu0.22.04.1
```

This audit does not substitute another 580.x build.

## RTX 5090 compatibility

**VERIFIED at the driver-support level.**

- The installed device is PCI ID `10de:2b85`.
- `ubuntu-drivers devices` offers `nvidia-driver-580-open` for this exact device.
- NVIDIA's 580.173.02 supported-chips document lists `NVIDIA GeForce RTX 5090`, device ID `2B85`.
- Local APT history and Isaac logs also show this same machine previously ran 580.173.02.

This establishes GPU support. It does not pre-prove every Isaac/CUDA/desktop runtime gate after a fresh reboot.

Evidence: [ubuntu_drivers_devices.txt](before/ubuntu_drivers_devices.txt), [official_580_supported_gpu.txt](before/official_580_supported_gpu.txt), [pci_devices.txt](before/pci_devices.txt).

Official source: https://us.download.nvidia.com/XFree86/Linux-x86_64/580.173.02/README/supportedchips.html#devid2B85

## Current 595 recovery package

The exact current package remains available from enabled Ubuntu repositories:

```text
nvidia-driver-595-open=595.84-0ubuntu0.22.04.1
```

APT reports the same version as installed and candidate. The recovery source is `jammy-updates/multiverse` and `jammy-security/multiverse`.

Package-level rollback status: **READY_ONLINE**. Exact `.deb` files were not downloaded or cached by this audit, so offline rollback is not claimed.

Evidence: [apt_recovery_595.txt](before/apt_recovery_595.txt), [apt_sources_full_redacted.txt](before/apt_sources_full_redacted.txt).

## Isaac5 baseline

- Current 595.84 evidence: SceneDB v4.4 reaches native plugin initialization and then the isolated probe segfaults; the renderer investigation narrows the existing backtrace to `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de`.
- Historical 580.173.02 evidence: the same Isaac Sim 5.1 installation initialized SceneDB v4.4, assigned the RTX viewport to device 0, and logged `Isaac Sim Full App is loaded` at 73.876 s.
- Isaac 5 GUI was not started during PHASE A.

Evidence: [isaac_log_baselines.txt](before/isaac_log_baselines.txt), [renderer analysis](../isaac5_renderer_viewport_analysis.md).

## Isaac6 baseline

- Historical 580.173.02 log: Isaac Sim 6.0.1 initialized SceneDB v6.1 and logged `Isaac Sim Full App is loaded` at 9.956 s.
- Current 595.84 GUI PASS is a user-supplied current fact, as already labeled in the renderer analysis; PHASE A did not relaunch Isaac 6 GUI to re-verify it.

Evidence: [isaac_log_baselines.txt](before/isaac_log_baselines.txt), [renderer analysis](../isaac5_renderer_viewport_analysis.md).

## CUDA/PyTorch baseline

- CUDA toolkit: `nvcc` 12.8.61 under `/usr/local/cuda-12.8`.
- The toolkit path and `nvcc` are not owned by a dpkg package; this appears to be an independently installed toolkit and is separate from the currently APT-managed display/kernel driver.
- System Python: `/usr/bin/python3`, Python 3.10.12.
- System Python has no `torch` module.
- Conda is not installed or not on PATH.
- `vulkaninfo` and `glxinfo` are not installed or not on PATH; no tools were installed for this audit.

Consequently, a post-switch PyTorch requirement cannot be compared in system Python. Unless the user names a project Python environment containing Torch, the honest Gate B12 baseline is `NOT AVAILABLE`, not FAIL.

Evidence: [cuda_python_baseline.txt](before/cuda_python_baseline.txt), [cuda_toolkit_provenance.txt](before/cuda_toolkit_provenance.txt), [graphics_baseline.txt](before/graphics_baseline.txt).

## Active GPU workloads

- `nvidia-smi --query-compute-apps`: no compute process listed.
- GPU graphics clients include Xorg, GNOME Shell, Awesun, and browser/application processes.
- No running Isaac process was observed at capture time.
- 24 long-running `closed_loop_demo_recorder.py` project processes were observed. They are not listed as NVIDIA compute clients, but a reboot would terminate them.

Status: **NO NVIDIA COMPUTE WORKLOAD, BUT ACTIVE DISPLAY/REMOTE GRAPHICS AND ACTIVE PROJECT PROCESSES**.

PHASE B must stop before mutation unless the user has deliberately preserved or ended the project processes and accepts the display interruption.

Evidence: [gpu_processes.txt](before/gpu_processes.txt), [project_processes.txt](before/project_processes.txt).

## Current kernel NVIDIA events

The current boot journal contains 112 occurrences of:

```text
NVRM: GPU0 refcntRequestReference_IMPL: Failed to enter state 1 ... status: 0x00000056
```

They span 2026-09-06 through 2026-09-09. No `NVRM: Xid` line was found by the recorded query, and current `nvidia-smi` succeeds. These events are not silently equated with Xid or hardware failure, but they mean the 595.84 baseline is not kernel-log-clean and contribute to the current HIGH risk rating.

Evidence: [kernel_nvidia_event_summary.txt](before/kernel_nvidia_event_summary.txt), [kernel_nvidia_events.txt](before/kernel_nvidia_events.txt).

## Disk space

Approximately 1.1 TiB is free on the filesystem containing both `/` and `/boot`. No cleanup, autoremove, or old-kernel deletion is needed or authorized.

## SSH recovery path

- `ssh.service`: enabled and active.
- Listening on IPv4 and IPv6 port 22.
- The current Codex process does not carry `SSH_CONNECTION` or `SSH_TTY`; it appears to be operating inside the local graphical session.
- Local interface addresses and routes were captured in [network_recovery_local.txt](before/network_recovery_local.txt), but are intentionally not repeated in this report.
- A successful login from a second device has not been tested.

Status: **SERVICE_READY / END_TO_END_UNVERIFIED**.

Before PHASE B, the user must verify either a real SSH login from another device or physical-console access. Awesun/ToDesk alone is not sufficient recovery proof because both can fail with the graphical driver.

## Rollback plan

Rollback must use the same Ubuntu APT/open-kernel method as the current driver. It must not use the historical `.run` installer.

If 580 has been installed and a reboot still leaves SSH or a TTY available:

```bash
sudo apt-get install --allow-downgrades \
  nvidia-driver-595-open=595.84-0ubuntu0.22.04.1
sudo reboot
```

After reboot:

```bash
nvidia-smi
modinfo -F version nvidia
dkms status
```

Required recovery observation:

```text
RTX 5090 visible
nvidia-smi Driver Version: 595.84
modinfo version: 595.84
nvidia/595.84 installed for the running kernel
```

Then run only the established Isaac 6 GUI smoke needed to recover the user-provided baseline. Do not run Isaac 5 first during rollback verification.

Rollback limitations:

- exact recovery packages are currently available online, not archived offline;
- external SSH authentication/reachability remains unverified;
- if networking and graphical login both fail, physical-console recovery is required.

## Package pinning decision

Ordinary `apt upgrade` does not normally switch from installed package name `nvidia-driver-580-open` to the different `nvidia-driver-595-open` metapackage. `ubuntu-drivers autoinstall`, a manual 595 install, or a future automation policy can switch branches.

For the short experiment window, the safest plan is:

- do not run `apt upgrade`, unattended driver-management commands, or `ubuntu-drivers autoinstall` between the 580 install and validation;
- do not add a hold before the driver test;
- if the final decision is `KEEP 580`, separately review a hold/pinning policy after the runtime gates, with explicit user approval.

No package was held during PHASE A.

## Risk classification

### Current: HIGH / NOT SWITCH-READY

Blocking operational gates:

1. External SSH login or physical-console recovery has not been proved.
2. 24 long-running project recorder processes would be lost on reboot unless intentionally handled.
3. Awesun/ToDesk and the active desktop depend on the GPU path being changed.

Additional risk factors:

- historical `.run` residue and its persistent Xorg configuration;
- 112 current-boot NVRM reference-state failures;
- CUDA 12.8 toolkit is independently installed and not dpkg-owned;
- 20-package NVIDIA branch replacement and reboot are unavoidable;
- no system-Python Torch baseline exists.

### Conditional: MEDIUM

Risk can be reduced to MEDIUM, not LOW, after all of the following are true immediately before mutation:

- user demonstrates a second-device SSH login or confirms physical-console access;
- user confirms the 24 recorder processes are safely preserved or may be terminated by reboot;
- `nvidia-smi` shows no compute task;
- a new APT dry-run matches the recorded NVIDIA-only 20-for-20 transaction;
- exact 595.84 recovery packages remain available;
- user explicitly says `继续切换580`.

## Exact proposed commands

These commands are a reviewed PHASE B proposal. **They were not executed.**

### 1. Final non-mutating gate

```bash
date --iso-8601=seconds
nvidia-smi
dkms status
uname -r
dpkg -l | rg -i 'nvidia|cuda'
ps -eo pid,ppid,user,etimes,stat,comm,args --sort=pid \
  | rg '/home/user/navigation_project/a_pipeline|/home/user/isaacsim/5\.1\.0|isaacsim-6\.0\.1'
hostname -I
systemctl is-enabled ssh
systemctl is-active ssh
apt-cache policy nvidia-driver-580-open nvidia-driver-595-open
LC_ALL=C apt-get -s --allow-downgrades install \
  nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1
```

Stop if any compute workload appears, recovery is not usable, the exact package disappears, or the transaction includes anything beyond the expected NVIDIA branch swap and necessary dependencies.

### 2. Targeted transition

```bash
sudo apt-get install --allow-downgrades \
  nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1
```

Do not use purge, autoremove, the `.run` installer, manual DKMS removal, `rmmod`, or a display-manager hot switch.

Before accepting APT's transaction, compare it against [apt_simulate_switch_580.txt](before/apt_simulate_switch_580.txt). The expected automatic removals are the 20 conflicting 595 NVIDIA packages; unrelated package removal is a STOP condition.

### 3. Clean module activation

```bash
sudo reboot
```

### 4. Post-reboot Gate 1

```bash
nvidia-smi
modinfo -F version nvidia
dkms status
```

If RTX 5090 or a 580.x driver is not visible, do not launch Isaac; enter the rollback procedure.

### 5. Existing graphics and CUDA tools only

```bash
nvcc --version
python3 - <<'PY'
import sys
print(sys.executable)
print(sys.version)
try:
    import torch
    print('torch', torch.__version__)
    print('cuda', torch.version.cuda)
    print('available', torch.cuda.is_available())
    if torch.cuda.is_available():
        print(torch.cuda.get_device_name(0))
except Exception as exc:
    print('torch check:', repr(exc))
PY
```

`vulkaninfo` and `glxinfo` are absent, so the plan must not install them without separate authorization.

### 6. Isaac 5 GUI gate

```bash
/home/user/isaacsim/5.1.0/isaac-sim.sh
```

Require a real visible 3D viewport, successful SceneDB initialization, `Isaac Sim Full App is loaded`, at least 60 seconds without native crash, and clean user-initiated teardown. Do not load Mecanum730 until this empty/default GUI gate passes.

### 7. Isaac 6 GUI gate

Only after Isaac 5 GUI and the bounded robot-only gate are reported:

```bash
/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1/isaac-sim.sh
```

Require a real visible 3D viewport and at least 60 seconds of stability. A log-only startup line does not substitute for the user's visual confirmation.

## STOP POINT

**STOPPED AFTER PHASE A.**

No NVIDIA driver or system package has been changed. No reboot or display-manager action has occurred. PHASE B remains forbidden until the user explicitly says `继续切换580` (or an equally explicit approval) and clears the recovery/process gates above.

### PHASE B continuation attempt at 2026-09-12 13:19 CST

The user subsequently authorized continuation. The mandatory final gate was rerun and **STOPPED FOR SAFETY before package mutation**:

- 24 long-running recorder processes remained active;
- SSH was listening but there was no established SSH session to prove the recovery route;
- the graphical session and remote-control clients remained active.

The candidate packages and dry-run transaction were unchanged and valid, and no NVIDIA compute client was present. The driver nevertheless remains 595.84 because the operational recovery/process gates were not cleared. See [phase_b_pre_switch_gate_20260912_131910.txt](logs/phase_b_pre_switch_gate_20260912_131910.txt).

The user later confirmed physical-console recovery and allowed the recorder processes to end on reboot. A subsequent `sudo -n true` check showed that the Codex process has no cached/noninteractive sudo authorization. The task stopped again before mutation; no password was requested and no install was attempted. See [phase_b_sudo_gate_20260912.md](logs/phase_b_sudo_gate_20260912.md).

### PHASE B package-install update at 2026-09-12 13:45 CST

The user subsequently authenticated locally and completed the exact targeted APT transition to `nvidia-driver-580-open=580.173.02-0ubuntu0.22.04.1`. Package health and DKMS checks passed, and the on-disk NVIDIA module is 580.173.02 for kernel `6.8.0-136-generic`. The current boot still has the former 595.84 module loaded, so the machine is intentionally stopped at the mandatory pre-reboot checkpoint. See [phase_b_post_install_pre_reboot_20260912_1345.md](logs/phase_b_post_install_pre_reboot_20260912_1345.md).
