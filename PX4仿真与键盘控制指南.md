用户速览
========

本指南适用于：

- PX4 工程：`/home/aim/px4_ws/PX4-Autopilot-v1.17.0`
- 仿真器：Gazebo Classic Iris
- 控制器：当前 PX4 树中已集成的 L1/DSun 自定义控制器
- 用途：仅用于 SITL 仿真，不要直接用于真实飞机

## 一、从新终端启动仿真

打开一个新的桌面终端（例如 GNOME Terminal），完整执行：

```bash
cd /home/aim/px4_ws/PX4-Autopilot-v1.17.0
source /home/aim/px4_ws/px4-venv/bin/activate
unset HEADLESS
unset NO_PXH
make px4_sitl gazebo-classic_iris
```

`make` 会在需要时先做增量编译，然后启动 PX4 和 Gazebo。此终端必须一直保持打开。

启动成功时应看到：

1. Gazebo Classic 窗口打开并显示 Iris 四旋翼。
2. 终端出现 `pxh>` 提示符。
3. PX4 输出 `Ready for takeoff!`。

在 `pxh>` 中执行以下命令可检查控制器：

```sh
mc_rate_control status
l1_adaptive_control status
```

正常结果：

- `mc_rate_control` 显示 `not running`。
- `l1_adaptive_control` 已运行，`subscriptions` 中的状态为 `1`。
- 飞机尚未起飞时，`armed=0`、`publish=0` 是正常现象。

> 专用 Iris 配置已经自动启动 `l1_adaptive_control` 并跳过 `mc_rate_control`。不要执行 `mc_rate_control stop`，也不要重复启动两个控制器。

## 二、开启键盘控制

等待 `Ready for takeoff!` 出现后，在同一个 `pxh>` 终端中依次执行：

```sh
param set SYS_FAILURE_EN 1
param set CA_FAILURE_MODE 1
l1_adaptive_control rc_control enable
manual_control stop
l1_keyboard_throttle start
```

前两个参数用于允许 Motor 1 故障注入；普通起降不会主动触发电机故障。

`manual_control stop` 用于防止 PX4 默认输入与键盘节点同时发布 `manual_control_setpoint`。

启动键盘节点后：

- 终端暂时不再显示 `pxh>`。
- 直接按键，不需要按回车。
- 必须使用启动 PX4 的这个真实交互终端；从另一个终端按键无法控制它。

## 三、键盘用法

| 按键 | 作用 |
|---|---|
| `1` | 请求解锁，解锁确认后自动起飞，并在离地约 1 米处悬停 |
| `2` | 平滑降落；接地确认后自动上锁 |
| `w` / `W` | 增大高度控制输入，提高目标高度 |
| `s` / `S` | 减小高度控制输入，降低目标高度 |
| `x` / `X` / 空格 | 把高度控制输入归零，保持当前目标高度 |
| `0` | 注入 Motor 1 完全失效，仅用于故障实验 |
| `r` / `R` | 恢复 Motor 1 |
| `q` / `Q` / `Esc` | 退出键盘节点并返回 `pxh>` |

### 正常起飞、悬停和降落

1. 短按一次 `1`，不要长按或连续按。
2. 等待终端显示解锁与起飞请求。
3. 飞机稳定悬停后，可用 `w`/`s` 调整高度，用空格停止继续上升或下降。
4. 按 `2` 开始降落。
5. 等待出现 `touchdown confirmed: vehicle disarmed`，不要提前按 `q`。
6. 确认已落地上锁后，按 `q` 返回 `pxh>`。

再次起飞前，先完全松开键盘并等待约 1 秒，然后只短按一次 `1`。

### Motor 1 故障实验

> 当前 `shut_m1_dsun_geometric` 实现在 Gazebo Classic Iris 中尚不能稳定完成三电机自旋悬停。按 `0` 后故障注入会生效，但控制分配可能把其余电机压到接近零并导致飞机快速下坠。不要把“释放偏航”理解为已经实现稳定自旋悬停。

如果只需要验证正常起飞、悬停和降落，请不要按 `0`。

进行故障实验时：

1. 只在 SITL 中使用，并先用 `1` 起飞至稳定悬停。
2. 按 `0` 关闭 Motor 1，准备好立即结束本轮仿真。
3. `r` 只负责撤销故障注入，不能保证已失稳的飞机恢复。
4. 不要将当前故障处理策略用于真机。

## 四、安全结束仿真

如果飞机已起飞：

1. 未进行电机故障实验时，按 `2` 降落。
2. 等待 `touchdown confirmed: vehicle disarmed`。
3. 按 `q` 退出键盘节点。
4. 回到 `pxh>` 后执行 `shutdown`。

如果已经按 `0` 并失稳，不要继续尝试普通降落流程；退出键盘节点后结束本轮 SITL，并重新启动仿真。

## 五、从另一个终端强制结束残留仿真

仅当原 PX4 终端已经无法操作时使用：

```bash
cd /home/aim/px4_ws/PX4-Autopilot-v1.17.0
build/px4_sitl_default/bin/px4-shutdown 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
```

确认残留进程已清理：

```bash
pgrep -a -x px4 || true
pgrep -a -x gzserver || true
pgrep -a -x gzclient || true
```

如果上面三条命令都没有输出，表示已经清理完成。

## 六、常见问题

### 终端没有出现 `pxh>`，或者立即显示 `Exiting NOW`

键盘节点需要真实交互 TTY。关闭本次仿真，换用 GNOME Terminal 等桌面终端重新执行第一节命令。

### 键盘节点启动后为什么没有 `pxh>`

这是正常现象。键盘节点正在独占读取当前终端；按 `q` 退出节点后才会重新出现 `pxh>`。

### 按键没有反应

确认 `l1_keyboard_throttle start` 是在启动 PX4 的同一个终端中执行的，并且按键不需要回车。

### 按 `1` 后没有起飞

先确认已出现 `Ready for takeoff!`，并用 `l1_adaptive_control status` 确认状态订阅有效。短按一次 `1`，不要长按或快速重复按键。

### Gazebo 已打开但仿真冻结

不要在运行时执行 `mc_rate_control stop`。这个工作区使用启动阶段的专用控制器所有权配置，正常情况下 `mc_rate_control` 从一开始就不会运行。

### 重启时提示端口占用

先按第五节清理残留仿真，确认 `px4`、`gzserver` 和 `gzclient` 都不存在后再重新启动。
