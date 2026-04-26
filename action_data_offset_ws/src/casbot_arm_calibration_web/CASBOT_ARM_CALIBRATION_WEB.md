# CASBOT 手臂位姿标定 Web 使用手册

## 1 系统概述

`casbot_arm_calibration_web` 是一个基于 ROS2 + Flask 的 Web 标定工具，提供浏览器界面对 CASBOT 机器人双臂进行：

- **笛卡尔空间位姿示教**（沿 base_link 系 X/Y/Z 平移、绕 RX/RY/RZ 旋转）
- **6D 偏移量保存**（位置 mm + 姿态 rad，外旋 XYZ 欧拉角）
- **单文件偏移轨迹生成与播放**（标定数据偏移生成播放）
- **多文件批量偏移轨迹生成与顺序播放**（乐队多片段数据偏移生成播放）

后端通过 `/joint_states` 话题获取关节角，使用 URDF + Pinocchio / TRAC-IK 进行正逆运动学计算，
示教指令以 100Hz 发布到 `/upper_body_debug/joint_cmd`（`UpperJointData`）。

---

## 2 编译与启动

### 2.1 编译（x86 Docker 环境示例）

```bash
# 加载 Docker 环境
./docker_env/docker_load_x86_aarch64.sh x86

# 编译
./docker_env/build.sh x86
```

### 2.2 启动

```bash
# source 安装空间
source install/setup.bash

# 启动 Web 节点（默认端口 6090）
ros2 launch casbot_arm_calibration_web arm_calibration_web.launch.py
```

浏览器打开 `http://<机器人IP>:6090/` 即可使用。

### 2.3 Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `host` | `0.0.0.0` | HTTP 监听地址 |
| `port` | `6090` | HTTP 端口 |
| `robot_urdf_path` | 空（使用包内默认 URDF） | URDF 文件路径；可为绝对路径或相对 urdf 目录的文件名 |
| `joint_states_topic` | `/joint_states` | JointState 话题名 |
| `joint_cmd_time_ref` | `1.0` | 示教按钮发布 UpperJointData 时的 time_ref（秒） |

自定义端口示例：

```bash
ros2 launch casbot_arm_calibration_web arm_calibration_web.launch.py port:=8080
```

### 2.4 Docker 挂载注意

Docker 容器内镜像可能存在 `/src/casbot_arm_calibration_web` 目录（非工作空间），
生成的偏移数据优先写入挂载的工作空间 `/workspace/<仓库名>/src/casbot_arm_calibration_web/new_offset_data/`。
若路径推断失败，可通过环境变量显式指定：

```bash
export CASBOT_CALIB_WORKSPACE_ROOT=/workspace/<仓库名>
# 然后重启节点
```

---

## 3 功能模块说明

### 3.1 界面布局

页面从上到下分为以下区域：

```
┌─────────────────────────────────────────────────┐
│                CASBOT 手臂位姿标定                │  ← 页面标题
├─────────────────────┬───────────────────────────┤
│       左手臂         │         右手臂             │  ← 双臂示教面板
│  ┌ 空间位置偏移 ┐   │  ┌ 空间位置偏移 ┐         │
│  │ X(前后)      │   │  │ X(前后)      │         │
│  │ Y(左右)      │   │  │ Y(左右)      │         │
│  │ Z(上下)      │   │  │ Z(上下)      │         │
│  └──────────────┘   │  └──────────────┘         │
│  ┌ 空间姿态偏移 ┐   │  ┌ 空间姿态偏移 ┐         │
│  │ RX(绕baseX)  │   │  │ RX(绕baseX)  │         │
│  │ RY(绕baseY)  │   │  │ RY(绕baseY)  │         │
│  │ RZ(绕baseZ)  │   │  │ RZ(绕baseZ)  │         │
│  └──────────────┘   │  └──────────────┘         │
├─────────────────────┴───────────────────────────┤
│  乐器（单选）  [ 开始标定 ] [ 结束标定 ]          │
│               [ 关闭上半身调试模式 ] [ 保存 ]      │  ← 控制区
├─────────────────────────────────────────────────┤
│  标定数据偏移生成播放                              │  ← 单文件偏移
│  数据来源 / 输出文件名 / 生成偏移数据 / 播放       │
├─────────────────────────────────────────────────┤
│  乐队多片段数据偏移生成播放                        │  ← 批量偏移
│  来源目录 / 多选文件 / 保存目录 / 批量生成         │
│  待播放列表（点击排序）/ 按顺序播放                 │
├─────────────────────────────────────────────────┤
│  说明事项                                        │  ← 页脚注释
└─────────────────────────────────────────────────┘
```

### 3.2 双臂位姿示教

每条手臂包含 6 个轴的示教控制：

**空间位置偏移**（X / Y / Z）：
- 步进值：默认 1 mm，可通过 ± 按钮以 0.5 mm 为单位调节，范围 0~10 mm
- 点击 `+` / `−` 按钮，手臂末端沿 base_link 对应轴平移
- 内部按 2 mm/路点展开子步，以 100Hz 发布，笛卡尔速度约 10 mm/s

**空间姿态偏移**（RX / RY / RZ）：
- 步进值：默认 0.1°，可调节 0.1°~10.0°
- 点击 `+` / `−` 按钮，手臂末端绕 base_link 对应轴旋转（保持位置尽量不变）
- 内部按 0.1°/子步展开，由 IK 解算实现

**刷新初始值**：
- 每个手臂面板右上角有「刷新初始值」按钮
- 点击后将当前末端 FK 位姿写入「初始值」字段
- 偏移量 = 当前值 − 初始值（位置为减法，姿态为相对旋转的旋转向量分量）

### 3.3 乐器选择与标定按钮

**乐器下拉（单选）**：架子鼓 / 贝斯 / 吉他 / 电子琴

| 按钮 | 功能 |
|------|------|
| **开始标定** | 开启上半身调试模式 → 播放对应乐器的 `resource/<乐器>/<乐器>_start_calibration.data` 轨迹 → 播完 1s 后自动采样写入左右臂初始值 |
| **结束标定** | 播放 `<乐器>_end_calibration.data` 轨迹（不自动关闭调试模式） |
| **关闭上半身调试模式** | 调用 `/motion/upper_body_debug` 服务关闭调试 |
| **保存** | 将当前左右臂末端 6D 偏移量（X/Y/Z mm + RX/RY/RZ rad 外旋 XYZ）写入 `web_saved_offsets.json` |

### 3.4 标定数据偏移生成播放（单文件）

**数据来源**（下拉选择）：

| 选项 | 说明 |
|------|------|
| `resource 开始标定` | 与「开始标定」使用同一 resource 数据 |
| `resource 结束标定` | 与「结束标定」使用同一 resource 数据 |
| `action_data_offset / data` | action_data_offset 包安装后的 data/ 目录 |
| `本包 offest_data / new_offset_data` | 包内已有偏移数据或新生成的偏移数据 |
| `自定义绝对路径` | 手动输入 .data 文件绝对路径 |

**生成偏移数据**：
- 读取已保存的 6D 偏移量（`web_saved_offsets.json`），若无则使用当前实时偏移
- 调用 `arm_trajectory_transform_jacobian`（Pinocchio 后端）生成 `<stem>_web_offset_YYYYMMDD_HHMMSS.data`
- 主副本写入 `new_offset_data/` 目录

**播放所选轨迹**：
- 先开启上半身调试模式，再播放当前数据来源下选中的轨迹

### 3.5 乐队多片段数据偏移生成播放（批量）

用于一次性对多个动作数据文件进行偏移计算，并按自定义顺序串接播放。

**数据来源目录**：填写包含 `.data` / `.csv` 动作数据文件的绝对路径。

**保存目录**：生成的偏移数据保存位置，留空则使用默认 `new_offset_data/` 目录。

**批量生成偏移数据**：对来源目录中选中的多个文件逐一进行偏移变换。

**待播放列表**：
- 点击条目添加到播放队列（左侧显示序号），再次点击取消
- 序号即为播放顺序
- 「清空顺序」清除所有已选
- 「扫描保存目录」扫描保存目录下的文件，补充到列表

**按顺序播放**：先开启上半身调试模式，再按序号顺序串接播放所有已选轨迹。

---

## 4 操作流程

### 4.1 单乐器标定完整流程

```
1. 启动 Web 节点
   └→ ros2 launch casbot_arm_calibration_web arm_calibration_web.launch.py

2. 打开浏览器访问 http://<IP>:6090/

3. 选择乐器（如"架子鼓"）

4. 点击「开始标定」
   └→ 弹窗确认 → 自动开启上半身调试 → 播放开始标定轨迹
   └→ 播完 1s 后自动采集初始值

5. 示教调整（重复直到满意）
   ├→ 空间位置偏移：X/Y/Z ± 按钮微调末端位置（步进值可调，单位 mm）
   └→ 空间姿态偏移：RX/RY/RZ ± 按钮微调末端姿态（步进值可调，单位 度）

6. 点击「保存」
   └→ 弹窗确认 → 偏移量写入 web_saved_offsets.json

7. 点击「生成偏移数据」
   └→ 弹窗确认 → 进度条显示 → 生成偏移轨迹文件
   └→ 成功弹窗显示文件名与路径

8. 验证偏移效果
   ├→ 数据来源切换到「本包 offest_data / new_offset_data」
   ├→ 选择刚生成的文件
   └→ 点击「播放所选轨迹」→ 观察机器人动作是否符合预期

9. 点击「结束标定」
   └→ 播放结束标定轨迹（不关闭调试模式）

10. 点击「关闭上半身调试模式」
    └→ 退出调试模式，机器人恢复正常控制
```

### 4.2 乐队多片段批量偏移流程

```
1. 前置条件：已完成步骤 4.1 的 1~6（已保存好偏移量）

2. 在「乐队多片段数据偏移生成播放」面板中操作：

3. 填写「数据来源目录」
   └→ 如 /workspace/band_songs/DZQ/raw_data
   └→ 点击「列出来源」→ 多选文件列表出现

4. 多选动作数据文件
   └→ 按住 Ctrl/Cmd 或 Shift 多选需要偏移的 .data 文件

5. 填写「保存目录」（可选）
   └→ 如 /workspace/band_songs/DZQ/offset_output
   └→ 留空则保存到默认 new_offset_data/ 目录

6. 点击「批量生成偏移数据」
   └→ 弹窗确认（显示来源目录、保存目录、文件数量）
   └→ 进度条显示 → 逐文件生成偏移轨迹
   └→ 完成后弹窗显示成功/失败统计

7. 在「待播放列表」中点击条目安排播放顺序
   ├→ 第 1 次点击 → 编号 ①，加入队列末尾
   ├→ 第 2 次点击另一条目 → 编号 ②
   ├→ 再次点击已选条目 → 取消选择，后续序号自动前移
   └→ 「清空顺序」清除所有选择

8. 点击「按顺序播放」
   └→ 弹窗确认 → 开启上半身调试 → 按 ①②③... 顺序串接播放
   └→ 播放期间所有标定按钮禁用

9. 播放完成后按需关闭调试模式
```

---

## 5 API 接口参考

### 5.1 状态与示教

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/state` | 获取当前左右臂末端位姿（位置 m + 姿态旋转向量分量 °）、初始值、偏移量 |
| POST | `/api/arm/<side>/axis/<axis>/adjust` | 单臂沿 base_link X/Y/Z 直线示教 |
| POST | `/api/arm/<side>/rotate/<axis>` | 单臂绕 base_link RX/RY/RZ 姿态示教 |
| POST | `/api/arm/<side>/initial/refresh` | 手动刷新指定臂的初始值 |

### 5.2 标定与调试

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/calibration/start` | 开始标定（开启调试 + 播放开始轨迹） |
| POST | `/api/calibration/end` | 结束标定（播放结束轨迹） |
| GET | `/api/calibration/status` | 查询当前播放状态 |
| POST | `/api/calibration/play_selected` | 播放当前数据来源下选中的轨迹 |
| POST | `/api/upper_body_debug` | 开启/关闭上半身调试模式 |

### 5.3 偏移数据（单文件）

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/save` | 保存 6D 偏移量到 `web_saved_offsets.json` |
| POST | `/api/offset_data_produce` | 生成偏移轨迹文件 |
| POST | `/api/offset_data_produce/preview` | 预览生成路径（不执行生成） |
| GET | `/api/trajectory/action_data_files` | 列出 action_data_offset 包 data/ 下文件 |
| GET | `/api/trajectory/offset_data_files` | 列出本包 offest_data + new_offset_data 下文件 |
| GET | `/api/trajectory/recent_generated_offset_files` | 本次进程内生成的偏移文件 |

### 5.4 偏移数据（批量 / 乐队多片段）

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/band/list_dir` | 列出指定目录下 `.data` / `.csv` 文件名 |
| POST | `/api/band/produce` | 批量生成偏移轨迹（多文件） |
| POST | `/api/band/play_sequence` | 按指定路径顺序串接播放多个偏移轨迹 |

---

## 6 文件与目录结构

```
casbot_arm_calibration_web/
├── launch/
│   └── arm_calibration_web.launch.py     # ROS2 launch 文件
├── casbot_arm_calibration_web/
│   ├── web_node.py                       # Flask 后端 + ROS2 节点
│   ├── trajectory_sources.py             # 轨迹文件路径解析
│   ├── offset_jacobian.py                # 调用 arm_trajectory_transform_jacobian
│   ├── calibration_data.py               # 轨迹 CSV 读取、关节名定义
│   ├── arm_fk.py                         # URDF 正运动学
│   ├── arm_cartesian_ik.py               # 笛卡尔直线 / 姿态 IK
│   ├── kin_pin_trac.py                   # Pinocchio + TRAC-IK 封装
│   ├── diag_cartesian_ik.py              # IK 诊断工具
│   └── web/
│       ├── index.html                    # 前端页面
│       ├── app.js                        # 前端逻辑
│       └── style.css                     # 前端样式
├── resource/                             # 各乐器标定轨迹 .data
│   ├── drum/
│   ├── bass/
│   ├── guitar/
│   └── keyboard/
├── offest_data/                          # 已有偏移数据
├── new_offset_data/                      # 新生成偏移数据（生成后出现）
├── web_saved_offsets.json                # 已保存 6D 偏移量（保存后出现）
├── urdf/                                 # 机器人 URDF 模型
├── setup.py
└── package.xml
```

---

## 7 调试示例

### 7.1 确认节点正常启动

```bash
# 终端输出应包含：
# CASBOT 手臂位姿标定 Web: http://0.0.0.0:6090/ （resource: .../resource）
# 末端 FK 已加载 (pinocchio): ...urdf，左臂 N 关节，右臂 N 关节

ros2 launch casbot_arm_calibration_web arm_calibration_web.launch.py
```

### 7.2 确认 /joint_states 话题在发布

```bash
# 应看到持续刷新的关节角度
ros2 topic echo /joint_states --once
```

若无输出，Web 界面位姿显示为 `—`，示教按钮会报错"末端位姿未就绪"。

### 7.3 测试 API（无需浏览器）

```bash
# 查看当前状态
curl http://localhost:6090/api/state | python3 -m json.tool

# 测试保存偏移量
curl -X POST http://localhost:6090/api/save \
  -H "Content-Type: application/json" \
  -d '{}' | python3 -m json.tool

# 查看 action_data 文件列表
curl http://localhost:6090/api/trajectory/action_data_files | python3 -m json.tool

# 查看 offset_data 文件列表
curl http://localhost:6090/api/trajectory/offset_data_files | python3 -m json.tool

# 列出某目录下的数据文件
curl -X POST http://localhost:6090/api/band/list_dir \
  -H "Content-Type: application/json" \
  -d '{"dir": "/workspace/band_songs/DZQ/raw_data"}' | python3 -m json.tool
```

### 7.4 示教失败排查

**现象**：点击 X/Y/Z 的 `+`/`-` 按钮，弹出"IK 只能沿请求方向达成 …%"

**原因**：手臂处于奇异位形或关节撞到限位，IK 无法达成请求方向的位移。

**解决**：
1. 检查终端日志中 `[IK]` 开头的诊断信息，关注"起始已贴近限位"提示
2. 先手动示教到远离限位的姿态（如肘弯曲 ~90°），再继续平移示教

**现象**：示教后机器人整体下沉 1~2cm

**原因**：首次示教时使用了 `/joint_states`（含重力 sag）作为基线。

**解决**：先播放一次「开始标定」轨迹，让控制器锁定姿态后再示教。

### 7.5 偏移数据生成失败排查

**现象**：点击「生成偏移数据」后弹窗"失败"

**排查步骤**：
```bash
# 1. 查看终端 [ERROR] 日志
#    常见："arm_trajectory_transform_jacobian 失败: ..."

# 2. 确认 action_data_offset 包已安装
ros2 pkg prefix action_data_offset

# 3. 确认 URDF 路径可达
ls -la $(ros2 pkg prefix action_data_offset)/share/action_data_offset/urdf/

# 4. 手动测试 jacobian 命令
ros2 run action_data_offset arm_trajectory_transform_jacobian -- \
  --robot-model /path/to/robot.urdf \
  --data /path/to/input.data \
  --output /tmp/test_output.data \
  --arm both \
  --offset-left="0,0,0,0,0,0" \
  --offset-right="0,0,0,0,0,0" \
  --length-unit mm \
  --angle-unit rad \
  --compose world_ee
```

**常见错误**：

| 错误信息 | 原因 | 解决 |
|----------|------|------|
| `需要 action_data_offset 包` | 未 source 环境 | `source install/setup.bash` |
| `URDF 路径解析失败` | URDF 文件不存在 | 检查 urdf 目录或指定 `robot_urdf_path` 参数 |
| `--offset-left: expected one argument` | 旧版代码负数参数解析问题 | 已修复：使用 `--offset-left=VALUE` 格式 |
| `未找到已保存偏移，且实时偏移不可用` | 未保存偏移且初始值未设置 | 先「开始标定」+ 示教 +「保存」 |

### 7.6 批量生成偏移数据失败排查

```bash
# 确认来源目录存在且包含 .data 文件
ls /workspace/band_songs/DZQ/raw_data/*.data

# 确认保存目录可写（或留空使用默认）
mkdir -p /workspace/band_songs/DZQ/offset_output

# 查看后端日志
# 终端会输出每个文件的生成结果：
# [Web][band] 批量生成: body=...
# [band] <filename> 生成失败: <error>
```

### 7.7 new_offset_data 目录下看不到文件

**排查**：

```bash
# 1. 确认生成成功弹窗中显示的路径
#    例如："文件路径：/workspace/.../new_offset_data/xxx.data"

# 2. 检查实际写入位置
find / -name "*_web_offset_*.data" -newer /tmp -type f 2>/dev/null

# 3. 在 Docker 环境中，确认挂载关系
# 容器内 /workspace/<仓库名>/ 应指向主机工作目录

# 4. 如路径推断错误，显式指定工作空间根
export CASBOT_CALIB_WORKSPACE_ROOT=/workspace/<仓库名>
# 然后重启节点
```

### 7.8 web_saved_offsets.json 内容查看

```bash
# 查看当前保存的偏移量
cat $(ros2 pkg prefix casbot_arm_calibration_web)/share/casbot_arm_calibration_web/web_saved_offsets.json | python3 -m json.tool

# 输出示例：
# {
#   "saved_at": "2026-04-26T21:00:00",
#   "unit": { "length": "mm", "angle": "rad", "euler_seq": "XYZ" },
#   "left":  { "x": 1.23, "y": -0.45, "z": 0.67, "rx": 0.001, "ry": -0.002, "rz": 0.003 },
#   "right": { "x": -1.23, "y": 0.45, "z": -0.67, "rx": -0.001, "ry": 0.002, "rz": -0.003 }
# }
```

---

## 8 说明事项

- 上方**外旋 XYZ 欧拉角偏移（度）**（与「保存」一致）。下方 RX/RY/RZ 为**旋转向量分量（度）**，与欧拉角数值一般不同；纯绕某一 base 轴小角度时通常对应轴的旋转向量接近指令角。
- 「保存」将界面左右臂 6D 偏移量（X/Y/Z 毫米 + RX/RY/RZ 弧度，外旋 XYZ）写入 `web_saved_offsets.json`；「生成偏移数据」优先读该 JSON（请先「保存」再生成）。
- 「开始标定」会先开启上半身调试再播放开始轨迹；「结束标定」仅播放结束标定轨迹；「关闭上半身调试模式」单独调用 `/motion/upper_body_debug`。
- 「播放所选轨迹」与「按顺序播放」都会先开启上半身调试再播放轨迹。
- 生成偏移数据时，主副本优先写入 `<工作空间>/src/casbot_arm_calibration_web/new_offset_data/`，并复制到 `install/.../share/.../new_offset_data/`。
- 批量生成支持自定义保存路径，留空则使用默认 `new_offset_data/` 目录。
