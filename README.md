# 重力补偿

## 安装匹诺曹库

```bash
sudo apt install ros-$ROS_DISTRO-pinocchio ros-$ROS_DISTRO-hpp-fcl ros-$ROS_DISTRO-coal
```

## 激活CAN模块

```bash
bash can_activate.sh
```

## 运行程序

运行前需先安装新的SDK，SDK代码仓库：[agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)

```bash
python3 main.py
```

## 程序默认参数（需根据实际情况修改）

- `力矩系数`：1.8-3及以上固件版本的系数
- `机械臂类型`：piper_x无夹爪版本
- `机械臂底座安装姿态`：水平安装
