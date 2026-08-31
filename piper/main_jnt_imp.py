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

from controller.jnt_imp_controller import JointImpedanceController


def main():
    # 定义机械臂模型的URDF文件路径，根据末端执行器的安装情况修改
    urdf_path = str(PROJECT_ROOT / "piper" / "piper" / "urdf" / "piper_description.urdf")

    # 控制频率
    control_frequency = 200.0

    # 初始化机械臂接口
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
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
    # robot.move_j([0.0, 0.2, -0.2, 0.0, 0.0, 0.0])
    # time.sleep(1)

    # 获取当前关节角度（等待首帧有效数据）
    joint_angles = None
    while joint_angles is None:
        js = robot.get_joint_angles()
        if js is not None:
            joint_angles = np.array(js.msg)
            break
        time.sleep(0.01)

    # 初始化关节阻抗控制器（基于 Pinocchio）
    controller = JointImpedanceController(
        urdf_path=urdf_path,
        dofs=robot.joint_nums,
    )

    # 每个关节单独设置阻尼 b 和刚度 k（按 J1~J6 顺序）
    b = np.array([0.5, 0.8, 0.8, 0.2, 0.2, 0.2], dtype=float)
    k = np.array([10.0, 10.0, 10.0, 2.0, 1.0, 1.0], dtype=float)
    if b.shape[0] != robot.joint_nums or k.shape[0] != robot.joint_nums:
        raise ValueError(f"b/k 长度必须等于关节数 {robot.joint_nums}")

    # 设置关节阻抗控制器参数
    controller.set_jnt_params(b, k)

    # 将当前位姿作为阻抗控制目标位姿（维持当前姿态）
    q_target = joint_angles.copy()
    v_target = np.zeros(robot.joint_nums)

    # 计算世界坐标系到基座坐标系的旋转矩阵
    roll, pitch, yaw = 0, 0, 0  # 单位：deg
    R_world_base = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

    print("开始重力补偿控制循环...")
    
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

            # 计算关节阻抗控制扭矩（含动力学补偿项）
            cmd_torque = controller.compute_jnt_torque(
                q_des=q_target,
                v_des=v_target,
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
    