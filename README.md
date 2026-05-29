# 动力学简单应用

## 安装匹诺曹库

```bash
sudo apt install ros-$ROS_DISTRO-pinocchio ros-$ROS_DISTRO-hpp-fcl ros-$ROS_DISTRO-coal
```

## 激活CAN模块

```bash
bash can_activate.sh
```

## 安装SDK

运行前需先安装新版 SDK：  
[agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)

## 项目结构

- `core/`：Pinocchio 封装、URDF/MDH 工具与解析层识别
- `controller/`：关节阻抗与笛卡尔阻抗控制器
- `nero/`：Nero 机型示例脚本
- `piper/`：Piper 机型示例脚本
- `piper_x/`：PiperX 机型示例脚本

## 运行示例

### Piper

```bash
python3 piper/main_gc.py             # 重力补偿
python3 piper/main_jnt_imp.py        # 关节阻抗
python3 piper/main_tast_imp.py       # 笛卡尔阻抗
```

### PiperX

```bash
python3 piper_x/main_gc.py
python3 piper_x/main_jnt_imp.py
python3 piper_x/main_tast_imp.py
```

### Nero

```bash
python3 nero/main_gc.py
python3 nero/main_jnt_imp.py
python3 nero/main_tast_imp.py
```
