import os
import numpy as np
import pinocchio as pin
from typing import Optional, Tuple


class helper:

    @staticmethod
    def as_vec(x: np.ndarray, dim: int, name: str) -> np.ndarray:
        """将输入转换为固定维度的一维向量。

        参数:
            x: 输入数据。
            dim: 目标维度。
            name: 参数名（用于报错提示）。

        返回:
            np.ndarray: 形状为 (dim,) 的浮点向量。

        异常:
            ValueError: 输入维度与 dim 不一致。
        """
        arr = np.asarray(x, dtype=float).reshape(-1)
        if arr.shape != (dim,):
            raise ValueError(f"{name} 维度必须为 ({dim},)")
        return arr

    @staticmethod
    def as_rot(rot: np.ndarray, name: str = "rot") -> np.ndarray:
        """将输入转换为 3x3 旋转矩阵。"""
        arr = np.asarray(rot, dtype=float)
        if arr.shape != (3, 3):
            raise ValueError(f"{name} 维度必须为 (3, 3)")
        return arr

    @staticmethod
    def orientation_error_rotmat(target_rot: np.ndarray, current_rot: np.ndarray) -> np.ndarray:
        """姿态误差（旋转矩阵输入，旋转向量输出）。"""
        r_t = helper.as_rot(target_rot, "target_rot")
        r_c = helper.as_rot(current_rot, "current_rot")
        r_err = r_t @ r_c.T
        return np.asarray(pin.log3(r_err), dtype=float).reshape(3).copy()


class AgxPinocchio:

    # ===== 初始化与模型状态 =====
    def __init__(self, urdf_path=None):
        """初始化 Pinocchio 机器人模型。

        参数:
            urdf_path: URDF 文件路径。

        异常:
            ValueError: urdf_path 为空。
        """
        if urdf_path is None:
            raise ValueError("urdf_path 不能为空")
        package_dirs = [os.path.dirname(os.path.dirname(urdf_path))]
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs)
        self.robot.data = self.robot.model.createData()
        self.nq = self.robot.model.nq
        self.nv = self.robot.model.nv
        self._default_gravity = self.robot.model.gravity.linear.copy()

    def _to_model_state(self, q: np.ndarray, v: Optional[np.ndarray] = None) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """将外部关节状态填充为模型维度。

        参数:
            q: 关节位置向量。
            v: 关节速度向量，可为 None。

        返回:
            Tuple[np.ndarray, Optional[np.ndarray]]: (q_full, v_full_or_none)。

        异常:
            ValueError: q 或 v 维度超出模型自由度。
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] > self.nq:
            raise ValueError(f"q 维度超出模型: {q.shape[0]} > {self.nq}")
        q_full = np.zeros(self.nq, dtype=float)
        q_full[:q.shape[0]] = q

        if v is None:
            return q_full, None

        v = np.asarray(v, dtype=float).reshape(-1)
        if v.shape[0] > self.nv:
            raise ValueError(f"v 维度超出模型: {v.shape[0]} > {self.nv}")
        v_full = np.zeros(self.nv, dtype=float)
        v_full[:v.shape[0]] = v
        return q_full, v_full

    def _set_gravity_from_base_orientation(self, base_orientation: Optional[np.ndarray]) -> np.ndarray:
        """根据基座姿态更新重力方向。

        参数:
            base_orientation: 基座姿态旋转矩阵 (3, 3)；为 None 时使用默认重力方向。

        返回:
            np.ndarray: 更新前的重力向量（用于调用方恢复）。

        异常:
            ValueError: base_orientation 格式非法。

        原理:
            若 R_wb 表示 base 相对 world 的旋转，则重力在 base 坐标系下为
            g_b = R_wb^T @ g_w。动力学计算始终在模型基坐标进行，
            因此需要先把 world 重力旋转到 base。
        """
        old_gravity = self.robot.model.gravity.linear.copy()
        if base_orientation is None:
            self.robot.model.gravity.linear = self._default_gravity.copy()
            return old_gravity

        rot = helper.as_rot(base_orientation, "base_orientation")

        gravity_world = self._default_gravity
        gravity_base = rot.T @ gravity_world
        self.robot.model.gravity.linear = gravity_base
        return old_gravity

    # ===== 运动学基础 =====
    def frame_id(self, frame_name: str) -> int:
        """按名称获取 frame id。

        参数:
            frame_name: 模型中的 frame 名称。

        返回:
            int: frame 对应的 id。
        """
        return self.robot.model.getFrameId(frame_name)

    def forward_kinematics(
        self,
        q: np.ndarray,
        frame_name: str,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """计算指定 frame 的位姿。

        参数:
            q: 关节位置向量。
            frame_name: 目标 frame 名称。

        返回:
            Tuple[np.ndarray, np.ndarray]: (position[3], rotation_matrix[3,3])。
        """
        q_full, _ = self._to_model_state(q)
        fid = self.frame_id(frame_name)
        pin.forwardKinematics(self.robot.model, self.robot.data, q_full)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        placement = self.robot.data.oMf[fid]
        return placement.translation.copy(), placement.rotation.copy()

    def jacobian(
        self,
        q: np.ndarray,
        frame_name: str,
        reference: int = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    ) -> np.ndarray:
        """计算指定 frame 的 6xN 几何雅可比。

        参数:
            q: 关节位置向量。
            frame_name: 目标 frame 名称。
            reference: 雅可比参考系（Pinocchio ReferenceFrame）。

        返回:
            np.ndarray: 指定 frame 的 6xN 几何雅可比。
        """
        q_full, _ = self._to_model_state(q)
        fid = self.frame_id(frame_name)
        jac_full = pin.computeFrameJacobian(self.robot.model, self.robot.data, q_full, fid, reference)
        dof = np.asarray(q).reshape(-1).shape[0]
        return jac_full[:, :dof].copy()

    # ===== 动力学核心 =====
    def nonlinear_effects(
        self,
        q: np.ndarray,
        v: np.ndarray,
        base_orientation: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """计算非线性项 n(q, qd)。

        参数:
            q: 关节位置向量。
            v: 关节速度向量。
            base_orientation: 基座姿态旋转矩阵 (3, 3)。

        返回:
            np.ndarray: 非线性项 n(q, qd) = C(q, qd) @ qd + g(q)。

        原理:
            该项包含速度相关项与重力项，是控制中常见的前馈补偿目标。
        """
        q_full, v_full = self._to_model_state(q, v)
        old_gravity = self._set_gravity_from_base_orientation(base_orientation)
        try:
            nle = pin.nonLinearEffects(self.robot.model, self.robot.data, q_full, v_full)
        finally:
            self.robot.model.gravity.linear = old_gravity
        return nle[: np.asarray(q).shape[0]]

    def inverse_dynamics(
        self,
        q: np.ndarray,
        v: np.ndarray,
        a: np.ndarray,
        base_orientation: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """计算逆动力学力矩 tau = M(q) @ qdd + C(q, qd) @ qd + g(q)。

        参数:
            q: 关节位置向量。
            v: 关节速度向量 qd。
            a: 关节加速度向量 qdd。
            base_orientation: 基座姿态旋转矩阵 (3, 3)。

        返回:
            np.ndarray: 与输入 q 等长的逆动力学力矩。

        原理:
            使用 RNEA 直接计算完整动力学项。该接口是动力学总入口，
            其中重力补偿对应 v=0 且 a=0 的特例。
        """
        q_full, v_full = self._to_model_state(q, v)
        a_vec = helper.as_vec(a, np.asarray(q).reshape(-1).shape[0], "a")
        a_full = np.zeros(self.nv, dtype=float)
        a_full[: a_vec.shape[0]] = a_vec
        old_gravity = self._set_gravity_from_base_orientation(base_orientation)
        try:
            tau = pin.rnea(
                self.robot.model,
                self.robot.data,
                q_full,
                v_full,
                a_full,
            )
        finally:
            self.robot.model.gravity.linear = old_gravity
        return tau[: np.asarray(q).shape[0]]
