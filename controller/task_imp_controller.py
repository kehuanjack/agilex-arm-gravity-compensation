import numpy as np
from typing import Optional
from core.agx_pinocchio import AgxPinocchio, helper


def orientation_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    """姿态误差（旋转向量），复用 agx_pinocchio 的统一实现。"""
    desired = np.asarray(desired, dtype=float)
    current = np.asarray(current, dtype=float)
    if desired.shape != (3, 3) or current.shape != (3, 3):
        raise ValueError("desired/current 必须是 3x3 旋转矩阵")
    return helper.orientation_error_rotmat(desired, current).astype(float)


class CartesianImpedanceController:
    """基于 Pinocchio 的笛卡尔空间阻抗控制（输出关节力矩）。"""

    def __init__(
        self,
        urdf_path: str,
        dofs: int,
        frame_name: str,
        b: Optional[np.ndarray] = None,
        k: Optional[np.ndarray] = None,
        joint_torque_weights: Optional[np.ndarray] = None,
    ):
        self.dofs = int(dofs)
        if self.dofs <= 0:
            raise ValueError("dofs 必须为正整数")
        if not frame_name:
            raise ValueError("frame_name 不能为空")

        self.frame_name = frame_name
        self.pin_model = AgxPinocchio(urdf_path)
        self.Bc = np.zeros(6, dtype=float)
        self.Kc = np.zeros(6, dtype=float)
        self.joint_torque_weights = np.ones(self.dofs, dtype=float)
        self.set_cart_params(
            b=np.array([5.0, 5.0, 5.0, 0.2, 0.2, 0.2], dtype=float) if b is None else b,
            k=np.array([200.0, 200.0, 200.0, 5.0, 5.0, 5.0], dtype=float) if k is None else k,
        )
        if joint_torque_weights is not None:
            self.set_joint_torque_weights(joint_torque_weights)

    def set_cart_params(self, b: np.ndarray, k: np.ndarray):
        """设置笛卡尔阻抗参数 Bc/Kc，均为长度6向量。"""
        b = np.asarray(b, dtype=float).reshape(-1)
        k = np.asarray(k, dtype=float).reshape(-1)
        if b.shape[0] != 6 or k.shape[0] != 6:
            raise ValueError("Bc/Kc 维度必须为6")
        self.Bc = b
        self.Kc = k

    def set_joint_torque_weights(self, weights: np.ndarray):
        """设置关节力矩权重，长度需等于 dofs。"""
        weights = np.asarray(weights, dtype=float).reshape(-1)
        if weights.shape[0] != self.dofs:
            raise ValueError(f"joint_torque_weights 维度必须等于 dofs={self.dofs}")
        self.joint_torque_weights = weights

    def compute_cartesian_torque(
        self,
        desired_pos: np.ndarray,
        desired_ori: np.ndarray,
        q_cur: np.ndarray,
        v_cur: np.ndarray,
        base_orientation: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        计算笛卡尔阻抗控制力矩：
            x_err = desired_x - current_x
            v_err = - J * v_cur
            F = Kc*x_err + Bc*v_err
            tau = J^T*F + nle

        desired_ori:
        - 3x3 旋转矩阵
        """
        q_cur = np.asarray(q_cur, dtype=float).reshape(-1)
        v_cur = np.asarray(v_cur, dtype=float).reshape(-1)
        desired_pos = np.asarray(desired_pos, dtype=float).reshape(-1)
        if q_cur.shape[0] != self.dofs or v_cur.shape[0] != self.dofs:
            raise ValueError(f"q_cur/v_cur 维度必须等于 dofs={self.dofs}")
        if desired_pos.shape[0] != 3:
            raise ValueError("desired_pos 维度必须为3")

        desired_ori = np.asarray(desired_ori, dtype=float)
        if desired_ori.shape == (3, 3):
            desired_rot = desired_ori
        else:
            raise ValueError("desired_ori 必须是 [3,3] 旋转矩阵")

        current_pos, current_rot = self.pin_model.forward_kinematics(q_cur, self.frame_name)
        
        j = self.pin_model.jacobian(q_cur, self.frame_name)

        nle = self.pin_model.nonlinear_effects(q_cur, v_cur, base_orientation)

        pos_error = desired_pos - current_pos
        ori_error = orientation_error(desired_rot, current_rot)

        x_error = np.concatenate([pos_error, ori_error])
        v_error = - j @ v_cur

        f_task = self.Kc * x_error + self.Bc * v_error

        tau = self.joint_torque_weights * (j.T @ f_task) + nle
        
        return tau
