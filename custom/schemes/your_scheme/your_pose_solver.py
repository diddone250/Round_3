# custom/schemes/your_scheme/your_pose_solver.py
"""你的位姿解算：一帧检测进，一帧观测出。

契约（core/interfaces/pose_solver.py）：
    输入     DetectionFrame + SelfState
    输出     Observation，每块看到的板一条 ArmorObservation

单板 PnP 在 custom/components/armor_solvers/，用它、换掉它、不用它都行。
坐标系约定：docs/coordinate_systems.md
"""
from __future__ import annotations

"""
@dataclass
class ArmorObservation:
    entity_type: EntityType          # 枚举类型 (Hero, Infantry, Rune 等)
    rel_pos: np.ndarray              # (3,) 浮点数组，[x, y, z] 坐标
    rel_rpy: np.ndarray              # (3,) 浮点数组，[roll, pitch, yaw] 姿态角
    armor_size: str = ""             # 字符串，"small" 或 "large"
    image_points_2d: Optional[np.ndarray] = None  # (4, 2) 二维数组，像素点
    lit: bool = False                # 布尔值，装甲板灯是否亮起
"""


import numpy as np
from core.bus.dataflow import Observation, ArmorObservation, DetectionFrame, SelfState
from core.interfaces.pose_solver import PoseSolver, register_pose_solver

from custom.components.armor_solvers import get_solver
from core.algorithms.math.transform import rpy_to_R


@register_pose_solver("yours")
class YourPoseSolver(PoseSolver):
    description = "我的方案：位姿解算_初版,仅仅调用已知的函数,而且仅仅解算装甲板的中心"

    def __init__(self):
        self.solver = get_solver("ippe")

    def solve(self, frame: DetectionFrame, self_state: SelfState) -> Observation:
        # TODO: 遍历 frame.detections，解出每块板的位置，打包成 Observation。
        obs_armors =[]

        #算出相机绝对姿态
        R_cam2abs = rpy_to_R(self_state.camera.rpy)

        for det in frame.detections :
            
            pos_C, rpy = self.solver.estimate(camera_k=self_state.camera.K,
                                              armor_size = det.armor_size,
                                              image_points_2d=det.image_points_2d,
                                              rot_camera_to_abs_camera=R_cam2abs,
                                              mount_pitch=0.0)
            if pos_C is None:
                continue

            com_pos_G = np.array(self_state.camera.pos) - np.array(self_state.gimbal.pos)
            rel_pos_G = com_pos_G + pos_C

            new_armor = ArmorObservation(
                entity_type = det.entity_type,
                rel_pos = rel_pos_G,
                rel_rpy = rpy,
                armor_size = det.armor_size)
            obs_armors.append(new_armor)

        return Observation(obs_armors, float(frame.timestamp))
