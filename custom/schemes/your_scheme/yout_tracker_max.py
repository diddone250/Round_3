# custom/schemes/your_scheme/your_tracker.py
from __future__ import annotations

import numpy as np
import math
import core.algorithms.math
from typing import List

#可以打大陀螺了,但是移动大陀螺打不了
from core.bus.dataflow import Observation, SelfState, Prediction
from core.interfaces.tracker import Tracker, register_tracker
from core.bus import public_bus, PublicMessages 
from core.algorithms.filters import ExtendedKalmanFilter
from config import cfg
from core.bus.dataflow import ShotSolution
from core.entities.property.entity_types import EntityType, TargetType

@register_tracker("max")
class YourTracker(Tracker):
    description = "我的方案：多板预测打分"

    def __init__(self):
        self.last_pos = None
        self.last_time = None
        
        # EKF 状态 [x, y, z, vx, vy, vz]
        self.ekf = ExtendedKalmanFilter(state_dim=6)
        self.Q = np.eye(6) * 0.05
        self.R = np.eye(3) * 0.1
        self.bullet_speed = cfg.bullet._17mm.speed
        
        # 云台平滑相关的变量
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.history_yaw = 0.0
        self.history_pitch = 0.0
        self.change = False
        
        # 系统延迟（发弹延迟+处理延迟），打大陀螺时可以尝试调大
        self.system_delay = 0.1 

        # 旋转状态相关的变量
        self.last_omega = 0.0
        self.last_armor_angle = None

    def reset(self):
        self.last_pos = None
        self.last_time = None
        self.ekf = ExtendedKalmanFilter(state_dim=6)
        self.history_yaw = 0.0
        self.history_pitch = 0.0
        self.change = False
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_omega = 0.0
        self.last_armor_angle = None

    def step(self, obs: Observation, self_state: SelfState) -> List[Prediction]:
        if obs is None or not obs.obs_armors:
            return []

        current_time = obs.timestamp
        
        #选出当前视野中最正对的装甲板（提供观测基准）
        best_armor = None
        max_point = -2.0
        for armor in obs.obs_armors:
            R_mat = core.algorithms.math.rpy_to_R(armor.rel_rpy)
            n_vector = R_mat[:, 0]
            sight_vector = -armor.rel_pos
            
            n_norm = n_vector / np.linalg.norm(n_vector)
            sight_norm = sight_vector / np.linalg.norm(sight_vector)
            point = np.dot(n_norm, sight_norm)
            
            if point > max_point:
                max_point = point
                best_armor = armor

        #确定车型参数（半径、板子夹角、高度偏移）
        if best_armor.entity_type == EntityType.Outpost:
            radius = 0.275
            armor_gap = 2.0 * math.pi / 3.0  # 前哨站3块板，夹角120度
            z_offset = best_armor.rel_pos[2] - 1.2 # 前哨站高低板的高度差修正(近似)
        elif best_armor.entity_type == EntityType.Hero:
            radius = 0.23
            armor_gap = math.pi / 2.0        # 英雄4块板，夹角90度
            z_offset = 0.0
        else:
            radius = 0.2
            armor_gap = math.pi / 2.0        # 步兵4块板，夹角90度
            z_offset = 0.0

        #计算当前【圆心】和【相位角】
        R_mat = core.algorithms.math.rpy_to_R(best_armor.rel_rpy)
        normal = R_mat[:, 0]
        
        # 圆心 = 板子位置 - 法向量 * 半径
        car_center = np.array(best_armor.rel_pos) - normal * radius
        current_angle = math.atan2(normal[1], normal[0])
        
        # 第一帧初始化
        if self.last_time is None:
            self.last_time = current_time
            self.last_pos = car_center
            self.ekf.x[0:3] = car_center
            self.last_armor_angle = current_angle
            return []

        dt = current_time - self.last_time
        if dt <= 1e-5:
            return []

        #计算旋转角速度 Omega
        delta_angle = current_angle - self.last_armor_angle
        # 处理角度过零跳变
        while delta_angle > math.pi: delta_angle -= 2 * math.pi
        while delta_angle < -math.pi: delta_angle += 2 * math.pi
        
        raw_omega = delta_angle / dt
        # 限制最大角速度，防止跨板瞬间造成的巨大误判
        if abs(raw_omega) > 12.0:
            raw_omega = 0.0  
        
        # 对角速度进行低通滤波，平滑输出
        self.last_omega = 0.7 * self.last_omega + 0.3 * raw_omega
        self.last_armor_angle = current_angle

        #EKF 只跟踪【车体中心】
        I_piece = np.eye(3)
        zeroI = np.zeros((3, 3))
        F = np.block([[I_piece, I_piece * dt], [zeroI, I_piece]])
        H = np.concatenate((I_piece, zeroI), axis=1)

        self.ekf.predict(F=F, Q=self.Q)
        
        # 位置突变极大时的宽容重置（不清空速度，保留平移惯性）
        jump = np.linalg.norm(car_center - self.last_pos)
        if jump > 0.5:
            self.ekf.x[0:3] = car_center
            self.ekf.P = np.eye(6) * 50
            self.change = True
            
        self.ekf.update(z=car_center, H=H, R=self.R)
        
        best_center = self.ekf.x[0:3]
        best_vel = self.ekf.x[3:6]
        
        self.last_time = current_time
        self.last_pos = car_center

        #大一统预测：未来多板打分
        muzzle_G = np.array(self_state.muzzle.pos) - np.array(self_state.gimbal.pos)
        dist = np.linalg.norm(best_center - muzzle_G)
        flight_time = dist / self.bullet_speed
        total_delay = flight_time + self.system_delay
        
        # 预测未来的圆心
        future_center = best_center + best_vel * total_delay
        
        # 维护 当前板、左板、右板 的未来候选角度
        candidate_angles = [
            current_angle + self.last_omega * total_delay,                     # 当前板未来的角度
            current_angle + self.last_omega * total_delay + armor_gap,         # 下一块板(左)未来的角度
            current_angle + self.last_omega * total_delay - armor_gap          # 上一块板(右)未来的角度
        ]

        best_score = -2.0
        best_aim_pos = None

        # 遍历这3个候选板的未来位置，打分
        for angle in candidate_angles:
            # 算出这块板未来的三维位置
            future_armor_pos = future_center + np.array([
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_offset
            ])
            # 算出这块板未来的法向量（朝向）
            future_normal = np.array([math.cos(angle), math.sin(angle), 0.0])
            
            # 算视线向量（从未来板子指向你的枪口）
            sight_vec = muzzle_G - future_armor_pos
            sight_norm = sight_vec / np.linalg.norm(sight_vec)
            
            # 打分：未来时刻，这块板朝向你的程度（越接近1，面积越大、越正对）
            score = np.dot(future_normal, sight_norm)
            
            if score > best_score:
                best_score = score
                best_aim_pos = future_armor_pos

        #云台解算 (基于选出的最佳未来板)
        d = best_aim_pos - muzzle_G
        delay = np.linalg.norm(d) / self.bullet_speed
        dx, dy, dz = d
        
        # 加上重力下坠补偿
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(dz + 0.5 * 9.8 * (delay**2), math.hypot(dx, dy))

        #云台平滑与前馈
        if self.history_pitch == 0.0 and self.history_yaw == 0.0:
            self.history_pitch = pitch
            self.history_yaw = yaw

        if self.change:
            smooth_yaw = yaw
            smooth_pitch = pitch
            self.history_yaw = yaw
            self.history_pitch = pitch
            self.change = False
        else:
            smooth_yaw = 0.85 * yaw + 0.15 * self.history_yaw
            smooth_pitch = 0.85 * pitch + 0.15 * self.history_pitch

        delta_yaw = smooth_yaw - self.last_yaw 
        while (delta_yaw > 3.13159): delta_yaw -= 6.28318
        while (delta_yaw < -3.13159): delta_yaw += 6.28318

        delta_pitch = smooth_pitch - self.last_pitch 
        while (delta_pitch > 3.13159): delta_pitch -= 6.28318
        while (delta_pitch < -3.13159): delta_pitch += 6.28318
        
        yaw_rate = np.clip((delta_yaw) / dt, -3.0, 3.0)
        pitch_rate = np.clip((delta_pitch) / dt, -3.0, 3.0)
        
        self.last_yaw = smooth_yaw
        self.last_pitch = smooth_pitch
        self.history_pitch = smooth_pitch
        self.history_yaw = smooth_yaw

        #火控判定
        # 只有当选出的那块“未来板”，在子弹到达时的正对程度超过 0.8，才开火
        fire = False
        if best_score > 0.8:
            fire = True

        #发送指令
        solution = ShotSolution(
            yaw=smooth_yaw, pitch=smooth_pitch,
            yaw_rate=yaw_rate, pitch_rate=pitch_rate,
            fire=fire, timestamp=current_time
        )
        public_bus().publish(PublicMessages.CONTROL, solution)

        if best_armor.entity_type == EntityType.Rune:
            tt = TargetType.RUNE
        elif best_armor.entity_type == EntityType.Outpost:
            tt = TargetType.OUTPOST
        else:
            tt = TargetType.ROBOT
            
        pred = Prediction(
            target_type=tt,
            center=future_center,
            armors=[best_aim_pos],
            timestamp=current_time,
            is_tracking=True
        )
        return [pred]