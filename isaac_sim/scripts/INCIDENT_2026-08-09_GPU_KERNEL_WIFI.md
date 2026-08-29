# 2026-08-09 Isaac Sim 主机卡死事故报告

## 结论

本次不是普通的应用卡顿、内存不足或网络断线。Isaac/RTX 负载触发 NVIDIA 驱动错误后，Linux 内核内存分配链发生 general protection fault，随后 CPU 10 的 `vmstat_update` 工作线程永久卡在自旋锁中。桌面、D-Bus 和网络请求都因此超时；用户主动重启是恢复动作，不是故障原因。

同时存在第二个独立问题：校园 Wi-Fi `suat` 在故障前频繁漫游和重新获取 DHCP 地址，会造成短时网络中断，但它发生在内核崩溃之前，不能解释 20:21 之后的整机锁死。

## 关键时间线与证据

- 2026-08-08 至 2026-08-09：Isaac/RTX 工作线程先后在 `librtx.hydra.so`、Fabric Scene Delegate、动画插件和 Carbonite 插件中出现多次随机 general protection fault。
- 2026-08-09 13:58:49：第一次内核 general protection fault，`ToDesk_Service` 触发 `d_path()` 解引用损坏的非规范指针。此前 6 分钟 Isaac 的 `carb.tasking1` 已崩溃。
- 18:43:51 起：上一轮开机共记录 26 次 `NVRM: refcntRequestReference_IMPL` 失败，并有一次 NVIDIA Xid 69。
- 20:20:58：PID 138078 启动 Isaac 6.0.1。
- 20:20:59：再次出现 NVIDIA refcount 错误。
- 20:21:04：5 秒后 PID 138078 在内核 `__rmqueue_pcplist()` 中触发第二次 general protection fault；寄存器包含损坏的链表/毒化指针。
- 20:21:55 至 22:19:31：CPU 10 持续 soft lockup，累计至少 6583 秒；日志中有 236 条 soft-lockup 和 40 条 RCU stall。
- 22:20：用户因整机卡死主动重启。
- 重启后的 580.173.02 驱动仍在每次 Isaac 启动约 1 秒后产生相同 refcount 错误；本轮开机已观察到 8 次，因此当前仍不允许继续跑 Isaac。

崩溃时不是资源耗尽：Isaac crash dump 显示仍有约 41.5 GiB RAM 和 1.79 GiB swap 可用；系统盘仅使用 34%。当前 CPU、GPU、NVMe 温度正常，也没有 OOM、NVMe I/O、PCIe AER、MCE 或 EDAC 硬件错误记录。

## 根因优先级

1. **NVIDIA 580 open 驱动与 RTX 5090/Isaac 6.0.1 的故障链（高）**：相同 NVRM 错误可以在干净重启后逐次复现，并在事故中领先内核 allocator fault 5 秒。当前 Ubuntu `ubuntu-drivers` 推荐 `nvidia-driver-595-open`，实际运行的是 `580.173.02-open`。
2. **i9-14900K/BIOS/内存稳定性（中高，必须排除）**：随机用户态库崩溃加上两个不同内核子系统的指针损坏，也符合 CPU/RAM 不稳定或既有 CPU 老化。主板 BIOS 为 1825；ASUS 已发布 1836 并明确说明更新 Intel 微码以增强稳定性。Intel 要求 13/14 代桌面 CPU 使用 Intel Default Settings 和含 0x12F 或更高微码的最新 BIOS。Ubuntu 当前加载的 OS 微码是 0x133，但 BIOS/ME 仍应升级并验证默认功耗设置。
3. **反复快速启动 Isaac（放大因素）**：事故前短时间内多次重启 Vulkan/RTX/PhysX，且每次几乎都伴随 refcount 错误，增加驱动状态损坏机会。
4. **Wi-Fi 漫游（独立网络问题）**：18:30–20:21 期间发生 27 次 AP 切换、6 次认证拒绝；20:17:12 后没有新的 Wi-Fi 事件，20:20:59 才出现 NVIDIA 错误，因此它不是内核卡死的触发源。

## 已实施的项目防护

`run_isaac_6_0_warehouse_people_robot.sh` 现在：

- 用 `flock` 原子互斥，杜绝两个启动器竞态启动。
- `nvidia-smi` 5 秒无响应、GPU 僵尸进程或当前开机已有 NVRM/Xid/GPF/soft-lockup/RCU-stall 时，安全退出码 6。
- 可用系统内存少于 16 GiB 时退出码 7。
- 清除外部 Python 3.10/ROS/CMake 路径；Isaac 只使用自身 Python 3.12 与 bundled Humble 库。
- 系统 ROS 2 `/cmd_vel` 由独立 Python 3.10 进程转发到 `127.0.0.1` UDP，避免把系统 `rclpy` ABI 加载进 Isaac Python 3.12。
- Isaac 在独立进程组运行；运行中每秒检查内核日志，新故障出现时立即 TERM/KILL 该组并返回退出码 6。
- 保持本地资产和 `ROS_LOCALHOST_ONLY=1`，仿真不依赖 Wi-Fi 或外网。

## 需要管理员/BIOS 权限完成的操作

1. 保存工作，升级 ASUS PRIME Z790-P WIFI BIOS 1825 → 1836。升级后载入 **Intel Default Settings**，先关闭 XMP/任何 CPU 或 GPU 超频进行稳定性验收。
2. 在 Ubuntu 中切换到发行版当前推荐的 595 open 驱动，然后重启：

   ```bash
   sudo apt update
   sudo apt install nvidia-driver-595-open
   sudo reboot
   ```

3. 重启后确认：

   ```bash
   nvidia-smi
   ubuntu-drivers devices
   journalctl -k -b 0 --no-pager | rg 'NVRM:|general protection fault|soft lockup|rcu: INFO'
   ```

4. 在 BIOS 更新、Intel 默认设置、XMP 关闭的状态下，从 GRUB 运行 Memtest86+ 至少 4 passes。若仍报错，分条内存测试；若内存无错但随机 GPF 继续，按 Intel 延长保修政策检查/RMA i9-14900K。
5. 固定工位优先接有线网口 `eno1`。若只能使用 Wi-Fi，可在确认不会移动机器后，把 `suat` 固定到当前最强的 5 GHz BSSID，并关闭 Wi-Fi 省电；该操作会牺牲 AP 自动切换能力，需单独确认后再执行。

## 资料

- NVIDIA Isaac Sim 6.0 要求（测试驱动 580.95.05）：https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html
- ASUS PRIME Z790-P WIFI BIOS 1836：https://www.asus.com/supportonly/prime%20z790-p%20wifi/helpdesk_bios/
- Intel 13/14 代桌面处理器 Vmin Shift 官方说明：https://www.intel.com/content/www/us/en/support/articles/000102331/processors.html
