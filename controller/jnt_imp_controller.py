import numpy as np
from typing import Optional
from core.agx_pinocchio import AgxPinocchio


class JointImpedanceController:
    """基于 Pinocchio 的关节阻抗控制力矩计算。"""

    def __init__(
        self,
        urdf_path: str,
        dofs: int,
        b: Optional[np.ndarray] = None,
        k: Optional[np.ndarray] = None,
    ):
        self.dofs = int(dofs)
        if self.dofs <= 0:
            raise ValueError("dofs 必须为正整数")

        self.pin_model = AgxPinocchio(urdf_path)
        self.B = np.zeros(self.dofs, dtype=float)
        self.K = np.zeros(self.dofs, dtype=float)
        self.set_jnt_params(
            b=0.2 * np.ones(self.dofs, dtype=float) if b is None else b,
            k=1.0 * np.ones(self.dofs, dtype=float) if k is None else k,
        )

    def set_jnt_params(self, b: np.ndarray, k: np.ndarray):
        """更新关节阻尼 B 与刚度 K。"""
        b = np.asarray(b, dtype=float).reshape(-1)
        k = np.asarray(k, dtype=float).reshape(-1)
        if b.shape[0] != self.dofs or k.shape[0] != self.dofs:
            raise ValueError(f"B/K 维度需等于 dofs={self.dofs}")
        self.B = b
        self.K = k

    def compute_jnt_torque(
        self,
        q_des: np.ndarray,
        v_des: np.ndarray,
        q_cur: np.ndarray,
        v_cur: np.ndarray,
        base_orientation: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        关节空间阻抗控制律（含动力学补偿）:
            tau = K(q_des-q) + B(v_des-v) + nle(q,v)
        其中 nle = C(q,v)v + g(q)。
        """
        q_des = np.asarray(q_des, dtype=float).reshape(-1)
        v_des = np.asarray(v_des, dtype=float).reshape(-1)
        q_cur = np.asarray(q_cur, dtype=float).reshape(-1)
        v_cur = np.asarray(v_cur, dtype=float).reshape(-1)

        if not all(x.shape[0] == self.dofs for x in (q_des, v_des, q_cur, v_cur)):
            raise ValueError(f"q/v 维度需全部等于 dofs={self.dofs}")
        
        nle = self.pin_model.nonlinear_effects(
            q_cur,
            v_cur,
            base_orientation=base_orientation,
        )
        acc_des = self.K * (q_des - q_cur) + self.B * (v_des - v_cur)
        tau = acc_des + nle
        return tau
