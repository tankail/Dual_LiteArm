# LiteArm 双臂上位机部署文档

本文档说明如何在一台新电脑上部署并运行：

```text
/home/tk/Dual_LiteArm/litearm_backend/backend_arms.sh
```

该脚本启动的是 LiteArm A10 左臂和右臂上位机，不包含腰部、头部等其他机构。

当前默认配置：

- 运行模式：`LIVE`
- Web 端口：`5000`
- Conda 环境：`panthera`
- 配置文件：`robot_param/litearm_arms.yaml`
- 左臂串口：通常为 `/dev/ttyACM0`
- 右臂串口：通常为 `/dev/ttyACM1`
- 控制周期：普通控制 `100 Hz`，重力补偿和阻抗控制 `200 Hz`

## 1. 工程目录要求

从 2026-08 起，`litearm_backend` 已改为**自包含**结构，不再依赖上一级的
`src/` 目录。部署时只需复制 `litearm_backend` 整个目录即可：

```text
litearm_backend/
├── app.py
├── backend.sh
├── backend_arms.sh
├── frontend/
├── litearm_python/           # 控制脚本 + motor_driver.py + litearm_demo_common.py
│   ├── motor_driver.py
│   └── litearm_demo_common.py
├── robot_param/
│   ├── litearm_arms.yaml
│   └── litearm_full.yaml
├── urdf/                     # 上位机自带的 URDF 和网格
│   ├── LiteArm_A10_251125.urdf
│   ├── LiteArm_A10_251224_left_arm.urdf
│   ├── LiteArm_A10_251224_right_arm.urdf
│   └── litearm_a10_251125/meshes/   # 3D 查看器用的 STL 网格
└── run_script.py
```

后端用到的所有外部资源（URDF、网格、`motor_driver.py`、
`litearm_demo_common.py`）都已复制到 `litearm_backend` 内部：

- `motor_driver.py` 位于 `litearm_python/`，`app.py`、`backend.sh`、
  `run_script.py` 都通过 `litearm_backend/litearm_python` 找到它。
- URDF 路径在 `robot_param/*.yaml` 中写成 `../urdf/...`，相对配置文件所在
  目录（`litearm_backend/robot_param`）解析，即指向 `litearm_backend/urdf/`。

因此整个 `litearm_backend` 目录可以整体复制到任意位置（不要求与 `src`
同级，也不要求放在 `/home/tk`）。运行时脚本会根据自身位置计算
`ROOT_DIR`。

## 2. 操作系统软件

以 Ubuntu 为例，先安装基础工具：

```bash
sudo apt update
sudo apt install -y git curl build-essential libgl1 libglib2.0-0
```

检查工具是否可用：

```bash
git --version
curl --version
```

## 3. 安装 Conda 和 Python 环境

`backend.sh` 默认查找名为 `panthera` 的 Conda 环境。如果新电脑尚未安装
Miniconda 或 Anaconda，需要先安装一个 Conda 发行版，然后重新打开终端。

创建环境：

```bash
conda create -n panthera python=3.10 -y
conda activate panthera
```

安装机器人动力学和数值计算依赖：

```bash
conda install -c conda-forge pinocchio numpy scipy pyyaml -y
```

安装串口、Web 服务和 WebSocket 依赖：

```bash
python -m pip install pyserial Flask Flask-SocketIO Flask-Cors
```

验证 Python 依赖：

```bash
python - <<'PY'
import flask
import flask_cors
import flask_socketio
import numpy
import pinocchio
import serial
import scipy
import yaml

print("LiteArm Python dependencies OK")
PY
```

### 更换 Conda 环境名

如果不想使用 `panthera`，可以使用其他环境：

```bash
export LITEARM_ENV_NAME=litearm
bash /home/tk/Dual_LiteArm/litearm_backend/backend_arms.sh
```

也可以把 `LITEARM_ENV_NAME` 写入 shell 配置文件。启动脚本会检查该环境
是否存在。

## 4. 安装前端构建环境

前端使用 Vite、Three.js 和 Socket.IO。新电脑需要安装 Node.js 18 或更高
版本，推荐 Node.js 20。

检查版本：

```bash
node --version
npm --version
```

如果 `node --version` 低于 `v18`，请先安装 Node.js 18 或更高版本，再继续
下面步骤。

构建前端：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend/frontend
npm ci
npm run build
```

构建成功后应生成：

```text
/home/tk/Dual_LiteArm/litearm_backend/frontend/dist/
```

后端会直接使用这个 `dist` 目录提供上位机页面。如果只启动 Python 后端而
没有构建前端，浏览器可能显示空白页面或旧页面。

## 5. 配置串口权限

连接两个电机通信板后检查串口：

```bash
ls -l /dev/ttyACM*
```

检查 Python 是否能发现串口：

```bash
conda run -n panthera python -c \
  "from serial.tools import list_ports; print([(p.device, p.description) for p in list_ports.comports()])"
```

将当前用户加入串口访问组：

```bash
sudo usermod -aG dialout "$USER"
```

执行后必须注销并重新登录，或者重启电脑，之后检查：

```bash
groups
```

输出中应包含：

```text
dialout
```

如果仍然出现 `Permission denied: /dev/ttyACM*`，优先检查是否已经重新登录。

### 串口映射说明

`litearm_arms.yaml` 当前配置为：

```text
/dev/ttyACM0 -> 本地电机 ID 1-8  -> 左臂全局 ID 1-8
/dev/ttyACM1 -> 本地电机 ID 1-8  -> 右臂全局 ID 9-16
```

配置中的 `serial.auto_resolve: true` 会扫描 `/dev/ttyACM*`，即使系统把
设备枚举为 `/dev/ttyACM2` 和 `/dev/ttyACM3`，程序也会按数字顺序尝试映射。

新电脑首次启动时必须核对启动日志中的左右臂是否对应正确。如果左右臂物理
对应关系反了，不要直接盲目交换电机 ID；应先固定 USB 接线顺序，必要时再
通过稳定的 udev 设备名或调整配置解决。

启动上位机前，不能同时运行其他会打开这些串口的程序，例如：

- 旧的 LiteArm 上位机
- 独立的重力补偿脚本
- 独立的阻抗控制脚本
- 电机调试工具
- 会占用 `/dev/ttyACM*` 的 ROS 节点

## 6. 启动前检查

执行以下检查：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend

chmod +x backend.sh backend_arms.sh

bash -n backend.sh
bash -n backend_arms.sh

test -f app.py
test -f robot_param/litearm_arms.yaml
test -f litearm_python/motor_driver.py
test -f urdf/LiteArm_A10_251125.urdf
test -f urdf/LiteArm_A10_251224_left_arm.urdf
test -f urdf/LiteArm_A10_251224_right_arm.urdf
```

检查 Python 文件是否存在语法问题：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend
conda run -n panthera python -m py_compile app.py litearm_python/litearm_control.py
```

## 7. 先运行 Demo 模式

首次部署建议先不连接或不驱动真实机械臂，验证前端和后端服务：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend
bash backend_arms.sh --demo --port 5000
```

浏览器打开：

```text
http://127.0.0.1:5000
```

如果从另一台电脑访问，把 `127.0.0.1` 替换为运行上位机电脑的 IP：

```text
http://<上位机IP>:5000
```

也可以在另一个终端检查 HTTP 服务：

```bash
curl -I http://127.0.0.1:5000
```

Demo 测试完成后，在运行窗口按 `Ctrl+C` 停止。

## 8. 运行真实双臂上位机

确认两个电机通信板已经连接，且没有其他程序占用串口后执行：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend
bash backend_arms.sh
```

脚本默认等价于：

```bash
bash backend.sh \
  --config robot_param/litearm_arms.yaml \
  --port 5000
```

启动成功后，日志中应看到类似内容：

```text
Starting LiteArm backend in LIVE mode on http://localhost:5000
Config: .../robot_param/litearm_arms.yaml
[Control] Starting at 100 Hz
[Broadcast] Starting at 30 Hz
Backend API: http://localhost:5000
WebSocket: ws://localhost:5000
Server starting on port 5000...
```

进入浏览器：

```text
http://localhost:5000
```

上位机的控制模式在后端进程内切换。运行上位机时，不需要同时启动
`litearm_python` 中的独立控制脚本，否则可能造成串口冲突。

## 9. 端口和远程访问

默认端口已经是 `5000`。也可以显式指定：

```bash
bash backend_arms.sh --port 5000
```

后端监听所有网卡。如果需要从局域网其他电脑访问，确认防火墙允许：

```bash
sudo ufw allow 5000/tcp
```

然后在浏览器访问：

```text
http://<上位机电脑的局域网IP>:5000
```

查看本机 IP：

```bash
ip addr
```

如果端口已经被占用：

```bash
ss -ltnp | grep ':5000'
```

可以先停止旧服务，或者临时使用其他端口：

```bash
bash backend_arms.sh --port 5001
```

使用其他端口时，浏览器也必须访问对应的端口。

## 10. 控制模式和配置说明

当前双臂配置使用：

```text
robot_param/litearm_arms.yaml
```

其中包含：

- 左右臂关节名、全局电机 ID 和关节限位
- 两个通信板的串口与电机类型
- 左右臂 URDF 路径
- 位置控制、重力补偿和阻抗控制周期
- 阻抗控制的 `kp`、`kd` 和力矩限制
- 末端外力/外力矩估计参数

阻抗控制的基本计算形式为：

```text
tau = G(q) + Kp * (q_target - q) - Kd * dq
```

其中：

- `G(q)` 是重力补偿力矩
- `Kp` 是位置误差刚度
- `Kd` 是速度阻尼
- `tau` 是最终关节力矩指令

修改 YAML 参数前，应先在低风险状态下验证，并确认没有超过电机和关节的
安全限制。

## 11. 常见问题排查

### 11.1 `conda: command not found`

脚本会自动尝试以下位置：

```text
~/miniconda3
~/anaconda3
/opt/conda
```

如果 Conda 已安装但不在这些位置，先手动加载：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate panthera
```

然后再次运行：

```bash
bash /home/tk/Dual_LiteArm/litearm_backend/backend_arms.sh
```

### 11.2 `conda env 'panthera' not found`

创建默认环境：

```bash
conda create -n panthera python=3.10 -y
```

然后按本文档第 3 节安装依赖。

### 11.3 `No module named pinocchio`

必须在启动脚本使用的同一个环境中安装：

```bash
conda install -n panthera -c conda-forge pinocchio -y
```

验证：

```bash
conda run -n panthera python -c "import pinocchio; print(pinocchio.__version__)"
```

### 11.4 串口找不到或无法打开

按顺序检查：

```bash
ls -l /dev/ttyACM*
groups
fuser -v /dev/ttyACM0 /dev/ttyACM1
```

重点确认：

1. 两个通信板 USB 连接正常。
2. 当前用户属于 `dialout` 组。
3. 没有旧进程占用串口。
4. 左右臂接线顺序与启动日志中的映射一致。

### 11.5 找不到 URDF 或 `motor_driver.py`

确认以下文件存在于 `litearm_backend` 内部：

```text
litearm_backend/litearm_python/motor_driver.py
litearm_backend/urdf/LiteArm_A10_251125.urdf
litearm_backend/urdf/LiteArm_A10_251224_left_arm.urdf
litearm_backend/urdf/LiteArm_A10_251224_right_arm.urdf
```

这些文件已随 `litearm_backend` 自包含。如果缺失，说明复制 `litearm_backend`
时遗漏了 `urdf/` 目录或 `litearm_python/motor_driver.py`。

### 11.6 页面空白或仍然显示旧版本

重新构建前端并重启后端：

```bash
cd /home/tk/Dual_LiteArm/litearm_backend/frontend
npm ci
npm run build
```

然后在浏览器执行强制刷新：

```text
Ctrl+Shift+R
```

### 11.7 后端启动但按键或控制模式没有反应

确认：

- 浏览器访问的是当前后端端口，而不是旧服务端口。
- 页面已连接 WebSocket。
- 终端没有出现串口读写错误。
- 没有同时运行独立控制脚本。
- 机械臂状态反馈正常，且没有触发关节限位或反馈安全保护。

## 12. 推荐部署顺序

新电脑上建议严格按下面顺序执行：

```bash
# 1. 复制 litearm_backend 目录（已自包含，无需上一级 src）

# 2. 创建 Conda 环境
conda create -n panthera python=3.10 -y
conda activate panthera

# 3. 安装 Python 依赖
conda install -c conda-forge pinocchio numpy scipy pyyaml -y
python -m pip install pyserial Flask Flask-SocketIO Flask-Cors

# 4. 安装 Node.js 18+，构建前端
cd /home/tk/Dual_LiteArm/litearm_backend/frontend
npm ci
npm run build

# 5. 配置串口权限并重新登录
sudo usermod -aG dialout "$USER"

# 6. 回到后端目录，先做 Demo 测试
cd /home/tk/Dual_LiteArm/litearm_backend
bash backend_arms.sh --demo --port 5000

# 7. Demo 正常后，停止 Demo，连接真实硬件
bash backend_arms.sh
```

首次真实运行时，建议让机械臂处于安全、可控的位置，并安排急停或断电
措施。启动成功后再逐项测试位置控制、重力补偿、阻抗控制、左右键盘控制和
轨迹功能。
