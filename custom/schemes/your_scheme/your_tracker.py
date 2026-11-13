# custom/schemes/your_scheme/your_tracker.py

"""你的跟踪器：一帧观测进，一帧预测出。

契约（core/interfaces/tracker.py）：
    输入     Observation + SelfState
    输出     List[Prediction]，第一条是"该打谁"
    没目标   返回空列表

云台只认 ShotSolution：要让云台动，得在 step() 里自己
public_bus().publish(PublicMessages.CONTROL, solution)。

写作指南：docs/writing_your_tracker.md
"""

from __future__ import annotations

"""
```python
@dataclass
class ArmorObservation:
    entity_type: EntityType
    rel_pos: np.ndarray              # (3,) G 系或 W 系（看 cfg.sim.frame）
    rel_rpy: np.ndarray              # (3,) 绝对系
    armor_size: str = ""
    image_points_2d: Optional[np.ndarray] = None
    lit: bool = False

@dataclass
class Observation:
    obs_armors: List[ArmorObservation] = field(default_factory=list)
    timestamp: float = 0.0

    
```
"""

#基础款,十分大的缺陷,但是好在好实现而且面对简单情况有不错效果
import numpy as np
import math
import core.algorithms.math
from typing import List

from core.bus.dataflow import Observation, SelfState, Prediction
from core.interfaces.tracker import Tracker, register_tracker
from core.bus import public_bus, PublicMessages 
from core.algorithms.filters import ExtendedKalmanFilter
from config import cfg
from core.bus.dataflow import ShotSolution
from core.entities.property.entity_types import EntityType, TargetType


@register_tracker("yours")
class YourTracker(Tracker):
    description = "我的方案：跟踪器（还没写）"
    def __init__(self):
        self.last_pos = None
        self.last_time = None
        self.velocity = np.array([0.0,0.0,0.0])
        self.ekf = ExtendedKalmanFilter(state_dim=6)
        self.Q = np.eye(6) *0.01
        self.R = np.eye(3) *0.1
        self.bullet_speed = cfg.bullet._17mm.speed
        self.last_yaw = 0.0
        self.last_pitch = 0.0 
        self.histoy_best_armor = None
        self.history_yaw = 0.0
        self.history_pitch = 0.0
        self.change = False
        self.system_delay = 0.1
        self.is_spinning = False
        self.trigger_reset = False
        self.chanbordt = 0.0

    def reset(self):
        self.last_pos = None
        self.last_time = None
        self.ekf = ExtendedKalmanFilter(state_dim=6)
        self.histoy_best_armor = None
        self.history_yaw = 0.0
        self.history_pitch = 0.0
        self.change = False
        self.last_yaw = 0.0
        self.last_pitch = 0.0

   
    def step(self, obs: Observation, self_state: SelfState) -> List[Prediction]:
        # TODO: 写你的跟踪器。obs.obs_armors 是本帧观测到的板（可能是空列表）。
        if obs is None or not obs.obs_armors :
            return []
        #算速度
        current_time = obs.timestamp

        
        self.best_armor = None
        self.max_point = -2.0
        for armor in obs.obs_armors:
            trun_matrix = core.algorithms.math.rpy_to_R(armor.rel_rpy)
            n_vector = trun_matrix[:,0]
            sight_vector = - armor.rel_pos
            #归一化
            norm = np.linalg.norm(n_vector)
            n_norm = n_vector / norm
            norm = np.linalg.norm(sight_vector)
            sight_norm = sight_vector / norm

            #打分
            point = np.dot(n_norm,sight_norm)
            if point > self.max_point:
                self.max_point = point
                self.best_armor = armor

        if self.histoy_best_armor is None :
            self.histoy_best_armor = self.best_armor

        jump = np.linalg.norm(self.histoy_best_armor.rel_pos - self.best_armor.rel_pos)
        self.histoy_best_armor = self.best_armor
        
        if jump > 0.3:
            self.ekf.x[0:3] = self.best_armor.rel_pos
            self.ekf.x[3:6] = [0.0,0.0,0.0]
            self.ekf.P = np.eye(6)*90
            self.change = True
            self.last_armor_yaw = self.best_armor.rel_rpy[2]
            d_cbt = current_time - self.chanbordt
            self.chanbordt = current_time
            if d_cbt < 0.8 and self.is_spinning == False:
                self.trigger_reset = True
                self.is_spinning = True
            if d_cbt > 0.85 and self.is_spinning == True:
                self.trigger_reset = True
                self.is_spinning = False

        d_tnc = current_time - self.chanbordt
        if d_tnc > 0.5 and self.is_spinning == True:
            self.trigger_reset = True
            self.is_spinning = False

        if self.last_time is None:
            self.last_time = current_time
            self.last_armor_yaw = self.best_armor.rel_rpy[2]
            
            # 必须给 last_pos 和 ekf.x 赋初值！否则第二帧必崩！
            self.last_pos = self.best_armor.rel_pos
            self.ekf.x[0:3] = self.best_armor.rel_pos
        
        dt = current_time - self.last_time

        if dt<= 1e-5:
            return []      

        #判断是否进入退出大陀螺
        """
        current_armor_yaw = self.best_armor.rel_rpy[2]
        delta_yaw = current_armor_yaw - self.last_armor_yaw
        while delta_yaw > 3.14159: delta_yaw -= 6.28318
        while delta_yaw < -3.14159: delta_yaw += 6.28318
        omega = abs(delta_yaw) / dt
        self.last_armor_yaw = current_armor_yaw

        if omega > 3.0 and self.is_spinning == False:
            self.is_spinning = True
            self.trigger_reset = True
        if omega < 2.6 and self.is_spinning == True:
            self.is_spinning = False
            self.trigger_reset = True
            """

        #大陀螺中心计算
        current_pos = self.best_armor.rel_pos
        if self.is_spinning == True:
            if self.best_armor.entity_type == EntityType.Outpost: radius = 0.275
            elif self.best_armor.entity_type == EntityType.Hero: radius = 0.23
            else: radius = 0.2
            R_mat = core.algorithms.math.rpy_to_R(self.best_armor.rel_rpy)
            normal = R_mat[:, 0]
            car_center = np.array(self.best_armor.rel_pos) - normal * radius
            current_pos = car_center

        I_piece = np.eye(3)
        zeroI = np.zeros((3,3))
        F_r = I_piece * dt
        F = np.block([[I_piece,F_r],[zeroI,I_piece]])
        H = np.concatenate((I_piece,zeroI),axis=1)
        self.ekf.predict(F = F , Q = self.Q)

        if(self.trigger_reset) :
            self.ekf.x[0:3] = current_pos
            self.ekf.x[3:6] = [0.0,0.0,0.0]
            self.ekf.P = np.eye(6) *100
            self.last_pos = current_pos
            self.trigger_reset = False
        raw_vel = (current_pos - self.last_pos) / dt
        if np.dot(self.ekf.x[3:6], raw_vel) <0:
            self.ekf.P = np.eye(6) *50
        
        self.last_time = current_time
        self.last_pos = current_pos

        self.ekf.update(z= current_pos , H =H, R= self.R)

        best_pos = self.ekf.x[0:3]
        best_vel = self.ekf.x[3:6]

        #枪口!=车
        muzzle_G = np.array(self_state.muzzle.pos) - np.array(self_state.gimbal.pos)
        d = best_pos - muzzle_G
        
        delay = np.linalg.norm(d) / self.bullet_speed
        aim_pos = best_vel * (delay+self.system_delay) + d
        dx,dy,dz = aim_pos
        yaw = math.atan2(dy , dx)
        pitch = math.atan2(dz + 0.5*9.8*(delay**2),math.hypot(dx,dy))

        if self.history_pitch is None:
            self.history_pitch = pitch
            self.history_yaw = yaw

        if self.change:
            # 切板瞬间：把历史直接强拉过来，确保前馈不爆炸，指令瞬间跳变
            smooth_yaw = yaw
            smooth_pitch = pitch
            self.history_yaw = yaw
            self.history_pitch = pitch
            self.change = False
        else:
            # 正常跟踪：低通滤波，平滑相机的微抖
            smooth_yaw = 0.85 * yaw + 0.15 * self.history_yaw
            smooth_pitch = 0.85 * pitch + 0.15 * self.history_pitch

        delta_yaw = smooth_yaw - self.last_yaw 
        while (delta_yaw> 3.13159):
            delta_yaw -= 6.28318
        while (delta_yaw< -3.13159):
            delta_yaw += 6.28318

        delta_pitch = smooth_pitch - self.last_pitch 
        while (delta_pitch> 3.13159):
            delta_pitch -= 6.28318
        while (delta_pitch< -3.13159):
            delta_pitch += 6.28318
        yaw_rate = (delta_yaw) / dt
        pitch_rate = (delta_pitch) / dt
        self.last_yaw = yaw
        self.last_pitch = pitch

        yaw_rate = np.clip(yaw_rate, -3.0, 3.0)
        pitch_rate = np.clip(pitch_rate, -3.0, 3.0)

        self.history_pitch = smooth_pitch
        self.history_yaw = smooth_yaw
        # 2. 你的决策算法，算出 solution (ShotSolution)
        # 假设这里你算出了一个 solution
        fire = True
        if self.is_spinning:
            enormal = normal / np.linalg.norm(normal)
            sight = -self.best_armor.rel_pos
            esight = sight / np.linalg.norm(sight)
            if np.dot(esight,enormal) > 0.8: fire = False

        solution = ShotSolution(
            yaw= smooth_yaw,pitch= smooth_pitch,
            yaw_rate= yaw_rate, pitch_rate= pitch_rate,
            fire= fire,timestamp= current_time
        )


        # 3. 把控制指令发出去（这行代码在 return 前面！）
        public_bus().publish(PublicMessages.CONTROL, solution)

        if self.best_armor.entity_type == EntityType.Rune:
            tt = TargetType.RUNE
        elif self.best_armor.entity_type == EntityType.Outpost:
            tt = TargetType.OUTPOST
        else:
            tt = TargetType.ROBOT
        pred_point = best_pos + best_vel * (delay+self.system_delay)
        pred = Prediction(
            target_type=tt,
            center=pred_point,
            armors=[pred_point],
            timestamp= current_time,
            is_tracking= True
        )
        return [pred]