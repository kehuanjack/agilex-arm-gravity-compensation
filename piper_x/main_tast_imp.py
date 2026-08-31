#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.task_imp_controller import CartesianImpedanceController


def main():
    # 定义机械臂模型的URDF文件路径，根据末端执行器的安装情况修改
    urdf_path = str(PROJECT_ROOT / "piper_x" / "piper_x" / "urdf" / "piper_x_description.urdf")

    # 控制频率
    control_frequency = 200.0

    # 初始化机械臂接口
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER_X,
        firmeware_version=PiperFW.V189,
        channel="can0",
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    # 等待机械臂使能
    while not robot.enable():
        time.sleep(1)
    print("机械臂使能成功")

    # 指定初始位置（根据需要修改）
    # robot.move_p([0.2, 0.0, 0.35, -1.57, 0.0, -1.57])
    # time.sleep(1)
    
    # 获取当前关节角度（等待首帧有效数据）
    joint_angles = None
    while joint_angles is None:
        js = robot.get_joint_angles()
        if js is not None:
            joint_angles = np.array(js.msg)
            break
        time.sleep(0.01)

    # 末端 frame 名称：按你的 URDF 实际末端 frame 修改
    ee_frame_name = "link6"

    # 初始化笛卡尔阻抗控制器（基于 Pinocchio）
    controller = CartesianImpedanceController(
        urdf_path=urdf_path,
        dofs=robot.joint_nums,
        frame_name=ee_frame_name,
    )
    
    # 关节力矩权重
    joint_torque_weights = np.array([1.0, 1.0, 1.0, 1.0, 0.5, 0.5], dtype=float)
    controller.set_joint_torque_weights(joint_torque_weights)

    # 笛卡尔空间阻尼/刚度（[x,y,z,rx,ry,rz]）
    b = np.array([5.0, 5.0, 5.0, 0.2, 0.2, 0.2], dtype=float)
    k = np.array([200.0, 200.0, 200.0, 5.0, 5.0, 5.0], dtype=float)
    if b.shape[0] != 6 or k.shape[0] != 6:
        raise ValueError("笛卡尔阻抗参数 b/k 长度必须为6")

    # 设置笛卡尔阻抗参数
    controller.set_cart_params(b, k)

    # 将当前末端位姿作为笛卡尔阻抗目标（维持当前末端位姿）
    x_target, r_target = controller.pin_model.forward_kinematics(joint_angles, ee_frame_name)

    # 计算世界坐标系到基座坐标系的旋转矩阵
    roll, pitch, yaw = 0, 0, 0  # 单位：deg
    R_world_base = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

    print("开始笛卡尔阻抗控制循环...")
    
    try:
        while True:
            start_time = time.time()

            # 获取当前关节角度和速度
            joint_angles = np.array(robot.get_joint_angles().msg)

            joint_velocities = np.zeros(robot.joint_nums)
            for i in range(1, robot.joint_nums + 1):
                ms = robot.get_motor_states(i)
                if ms is not None:
                    joint_velocities[i - 1] = ms.msg.velocity

            # 计算笛卡尔阻抗控制扭矩（含动力学补偿项）
            cmd_torque = controller.compute_cartesian_torque(
                desired_pos=x_target,
                desired_ori=r_target,
                q_cur=joint_angles,
                v_cur=joint_velocities,
                base_orientation=R_world_base,
            )

            # 应用重力补偿扭矩
            try: 
                for joint_id in range(1, robot.joint_nums + 1):
                    robot.move_mit(joint_id, 0, 0, 0, 0, cmd_torque[joint_id - 1])
                    
            except Exception as e:
                print(f"应用重力补偿失败: {e}")
            
            # 控制频率
            t = 1.0 / control_frequency
            elapsed_time = time.time() - start_time
            if elapsed_time < t:
                time.sleep(t - elapsed_time)
            else:
                print(f"警告：控制循环超时 {elapsed_time:.3f}s > {t:.3f}s")
                
    except KeyboardInterrupt:
        print("\n用户中断，停止重力补偿")
        for joint_id in range(1, robot.joint_nums + 1):
            try:
                robot.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
            except:
                pass

    except Exception as e:
        print(f"程序运行出错: {e}")
        for joint_id in range(1, robot.joint_nums + 1):
            try:
                robot.move_mit(joint_id, joint_angles[joint_id - 1], 0, 10, 0.8, 0)
            except:
                pass


if __name__ == "__main__":
    main()
    