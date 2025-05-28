# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class Go2CrawlCfg( LeggedRobotCfg ):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.42]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class terrain( LeggedRobotCfg.terrain ):
            vertical_scale = 0.005 # [m]
            border_size = 5 # [m]
            height = [0.01, 0.03]
            simplify_grid = False
            gap_size = [0.02, 0.1]
            stepping_stone_distance = [0.02, 0.08]
            downsampled_scale = 0.075
            curriculum = True
            terrain_dict = {"smooth slope": 0., 
                            "pyramid_sloped": 0.,
                            "discrete_obstacles": 0.2,
                            "stepping_stones": 0., 
                            "random_uniform_terrain": 0.8, 
                            "discrete": 0., 
                            "stepping stones": 0.,
                            "gaps": 0., 
                            "smooth flat": 0.,
                            "pit": 0.,
                            "wall": 0.0,
                            "platform": 0.,
                            "large stairs up": 0.,
                            "large stairs down": 0.,
                            "parkour": 0.,
                            "parkour_hurdle": 0.,
                            "parkour_flat": 0.,
                            "parkour_step": 0.,
                            "parkour_gap": 0.,
                            "demo": 0.0,}
            terrain_proportions = list(terrain_dict.values())
    class commands( LeggedRobotCfg.commands ):
        curriculum = True
        max_curriculum = 1.
        num_commands = 5 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 6. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        
        lin_vel_x_clip = 0.1
        lin_vel_y_clip = 0.05
        ang_vel_yaw_clip = 0.05
        # Easy ranges
        class ranges:
            lin_vel_x = [-0.3, 0.5] # min max [m/s]
            lin_vel_y = [-0.2, 0.2]   # min max [m/s]
            ang_vel_yaw = [-0.2, 0.2]    # min max [rad/s]
            heading = [0, 0]
            base_height = [0.15, 0.32]#TODO 修改为高度变化量指令

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 40.}  # [N*m/rad]
        damping = {'joint': 1.}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        # file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go1/urdf/go1_new.urdf'
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base","hip"]#, "thigh", "calf"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 1.0
        base_height_target = 0.25
        clearance_height_target = -0.22
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = 10.0
            tracking_lin_vel = 1.5
            tracking_ang_vel = 1.0

            bounds_loss_coef = 0.0
            locomotion_height = 2
            lin_vel_z = -0.0 #爬行任务把他设置为0
            ang_vel_xy = -0.1
            torques = -0.00001#惩罚总扭矩大小的平方
            delta_torques = 0.#扭矩大小变化的平方
            dof_vel = 0.
            dof_acc = 0.#扭矩速度变化的平方
            base_height = 0.0
            feet_air_time = 0.01#脚步悬空
            collision = -1.#所选位置的碰撞
            feet_stumble = 0.0
            action_rate = -0.001#惩罚动作的变化
            stand_still = -0.1
            dof_pos_limits = -0.0#惩罚关节位置太接近极限
            dof_vel_limits = -0.0  #惩罚关节速度太接近极限
            hip_pos = -0.0#惩罚hip位置偏离default的程度
            dof_error = -0.0#-0.1# 惩罚所有关节位置偏离default的程度
            torque_limits = -0.0#扭矩限制
            # ----add---
            orientation = -0.1#惩罚非水平姿态
            upward = 0.1#奖励直立状态
            foot_clearance_up = -0.#惩罚足部高度误差，和足端横向移动
            foot_mirror_up =-0.01 #镜像对称
            hip_abduction = 0.0#鼓励hip接近0度
class Go2RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_go2'
        max_iterations = 20000
  
