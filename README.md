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
