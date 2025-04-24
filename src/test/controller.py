from state_estimater import robotestimater
import threading
import rclpy
import math
import time
import numpy as np
import yaml
# import torch
import os
import pickle

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class RobotState:

    SELF_CHECK = "self_check"          # 自我檢測
    LOCKING_LEGS = "locking_legs"      # 鎖定腿部
    IDLE = "idle"                      # 空閒等待
    EMERGENCY_STOP = "emergency_stop"  # 緊急停止
    ERROR = "error"                    # 錯誤狀態
    RLCONTROL = "RLcontrol"
    
class robotcontroller:
    
    def __init__(self):
        
        rclpy.init()
        self.estimator = robotestimater()
        
        self.state = None  # 初始狀態為 None，表示尚未初始化
        self.previous_state = None
        
        #self_check檢查的部分
        self.self_check_passed = False  # 用來標記 self_check 是否成功
        self.self_check_step = 0  # 重置
        self.running_time = 0.0  # 重置
        self.inital_pos1 = 0.0
        self.inital_pos2 = 0.0
        self.degree_1 = 0.0
        self.degree_2 = 0.0
        
        # cfg_path = os.path.join('./pt_dir', 'cfgs.pkl')
        # if os.path.exists(cfg_path):
        #     print("文件存在:", cfg_path)
        #     self.env_cfg, self.obs_cfg, reward_cfg, command_cfg, curriculum_cfg, domain_rand_cfg, terrain_cfg, train_cfg = pickle.load(open(cfg_path, "rb"))
        # else:
        #     print("文件不存在:", cfg_path)
        #     exit()
        # # 加载模型
        # try:
        #     self.loaded_policy = torch.jit.load(os.path.join("./pt_dir", "policy.pt"))
        #     self.loaded_policy.eval()  # 设置为评估模式
        #     self.loaded_policy.to(device)
        #     print("模型加载成功!")
        # except Exception as e:
        #     print(f"模型加载失败: {e}")
        #     self.transition_to(RobotState.ERROR)
        # self.history_obs_buf = torch.zeros((self.obs_cfg["history_length"], self.obs_cfg["num_slice_obs"]), device=device, dtype=torch.float32)
        # self.slice_obs_buf = torch.zeros(self.obs_cfg["num_slice_obs"], device=device, dtype=torch.float32)
        # self.obs_buf = torch.zeros((self.obs_cfg["num_obs"]), device=device, dtype=torch.float32)
        # self.default_dof_pos = torch.tensor([self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["dof_names"]],
        # device=device,
        # dtype=torch.float32)
        # lower = [self.env_cfg["dof_limit"][name][0] for name in self.env_cfg["dof_names"]]
        # upper = [self.env_cfg["dof_limit"][name][1] for name in self.env_cfg["dof_names"]]
        # self.dof_pos_lower = torch.tensor(lower).to(device)
        # self.dof_pos_upper = torch.tensor(upper).to(device)
        # self.obs_scales = self.obs_cfg["obs_scales"]
        # self.commands_scale = torch.tensor(
        #     [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], 
        #     self.obs_scales["ang_vel"], self.obs_scales["height_measurements"]], 
        #     device=device, dtype=torch.float32
        # )
    
    def transition_to(self, new_state):
        if new_state != self.state:
            if new_state in [RobotState.EMERGENCY_STOP]:
                print(f"State transition: {self.state} -> {new_state}")
                self.previous_state = self.state
                self.state = new_state
                print("emergency stop")
                return
            elif not self.self_check_passed and new_state not in [RobotState.SELF_CHECK, RobotState.ERROR]:
                print("Error: Cannot transition to other states until self_check is successful!")
                return
            print(f"State transition: {self.state} -> {new_state}")
            self.previous_state = self.state
            self.state = new_state
            # self.state_transition_time = time.time()
    
    def update(self):
        """狀態機更新"""
        if self.state == RobotState.SELF_CHECK:
            self.handle_self_check()
        elif self.state == RobotState.LOCKING_LEGS:
            self.handle_locking_legs()
        elif self.state == RobotState.IDLE:
            self.handle_idle()
        elif self.state == RobotState.EMERGENCY_STOP:
            self.handle_emergency_stop()
        elif self.state == RobotState.ERROR:
            self.handle_error()
        elif self.state == RobotState.RLCONTROL:
            self.handle_rlcontrol()
        
    def handle_self_check(self):
        
        dt = 0.01
        process_time_1 = 3
        process_time_2 = 3
        imu_test_degree = 30
        first_cheak_pos = 0.97
        second_cheak_pos = first_cheak_pos + math.pi * (imu_test_degree / 180)

        if self.self_check_step == 0:
            self.estimator.unitree_sendrecv(free_joint_mode=True)
            if abs(self.estimator.get_joint_pos(1) - self.estimator.get_joint_pos(4)) <= 0.1:
                self.inital_pos1 = self.estimator.get_joint_pos(motor_id=1)
                self.inital_pos2 = self.estimator.get_joint_pos(motor_id=4)
                print(self.inital_pos1, self.inital_pos2)
                self.self_check_step = 1
            else:
                print("unitree_init_check_fail")
                self.transition_to(RobotState.ERROR)

        elif self.self_check_step == 1:
            self.running_time += dt
            phase = np.tanh(self.running_time / (process_time_1/3))
            self.estimator.set_motor_cmd(motor_id=1, kp=3, kd=0.1, position=phase * first_cheak_pos + (1-phase)*self.inital_pos1)
            self.estimator.set_motor_cmd(motor_id=2, kp=3, kd=0.1, position=self.estimator.get_joint_pos(motor_id=2))
            self.estimator.set_motor_cmd(motor_id=4, kp=3, kd=0.1, position=phase * first_cheak_pos + (1-phase)*self.inital_pos2)
            self.estimator.set_motor_cmd(motor_id=5, kp=3, kd=0.1, position=self.estimator.get_joint_pos(motor_id=5))
            self.estimator.unitree_sendrecv(free_joint_mode=False)
            if self.running_time >= process_time_1:
                self.self_check_step = 2
                self.degree_1, _dq = self.estimator.get_filter_pitch_orintation()
                print(self.degree_1)
                self.running_time = 0

        elif self.self_check_step == 2:
            self.running_time += dt
            phase = np.tanh(self.running_time / (process_time_2/3))
            self.estimator.set_motor_cmd(motor_id=1, kp=3, kd=0.1, position=phase * second_cheak_pos + (1-phase)*first_cheak_pos)
            self.estimator.set_motor_cmd(motor_id=2, kp=3, kd=0.1, position=self.estimator.get_joint_pos(motor_id=2))
            self.estimator.set_motor_cmd(motor_id=4, kp=3, kd=0.1, position=phase * second_cheak_pos + (1-phase)*first_cheak_pos)
            self.estimator.set_motor_cmd(motor_id=5, kp=3, kd=0.1, position=self.estimator.get_joint_pos(motor_id=5))
            self.estimator.unitree_sendrecv(free_joint_mode=False)
            if self.running_time >= process_time_2:
                degree_2, _dq = self.estimator.get_filter_pitch_orintation()
                if imu_test_degree - 1 < (abs(self.degree_1 - degree_2) / math.pi)*180  < imu_test_degree + 1:
                    print("Self-check passed!")
                    self.self_check_passed = True
                    self.transition_to(RobotState.IDLE)
                else:
                    print("imu_check_fail")
                    print(abs(self.degree_1 - degree_2) / 180)
                    self.transition_to(RobotState.ERROR)
                self.self_check_step = 0  # 重置狀態
                self.running_time = 0.0
            
            
    def handle_locking_legs(self):
        pass
    
    def handle_idle(self):
        pass
    
    def handle_emergency_stop(self):
        
        self.estimator.unitree_sendrecv(free_joint_mode=True)
    
    def handle_error(self):
        
        self.estimator.unitree_sendrecv(free_joint_mode=True)
    
    # def handle_rlcontrol(self):
        
    #     actions = self.loaded_policy(self.obs_buf)
    #     actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
    #     project_gravity = self.estimator.get_gravity_orientation()
    #     x_lin_vel = self.estimator.get_linear_vel_x_with_kalman()
    #     ang_vel_x,ang_vel_y,ang_vel_z = self.estimator.get_all_angular_velocity()
    #     dof_pos = torch.zeros(self.env_cfg["num_actions"]-2, device=device, dtype=torch.float32)
    #     dof_vel = torch.zeros(self.env_cfg["num_actions"], device=device, dtype=torch.float32)
    #     for i in range(4):
    #         dof_pos[i]=self.estimator.unitree_motor_state[i, 0]
    #         dof_vel[i]=self.estimator.unitree_motor_state[i, 1]
    #     for i in range(2):
    #         dof_vel[i+4]=self.estimator.foc_motor_status[i, 1]
    #     base_lin_vel = torch.tensor([x_lin_vel, 0.0, 0.0], device=device, dtype=torch.float32)
    #     base_ang_vel = torch.tensor([ang_vel_x,ang_vel_y,ang_vel_z], device=device, dtype=torch.float32)
    #     cmds = torch.tensor([0, 0, 0, 0.25], device=device, dtype=torch.float32)
    #     self.slice_obs_buf = torch.cat(
    #         [
    #             base_lin_vel*self.obs_scales["lin_vel"],
    #             base_ang_vel*self.obs_scales["ang_vel"],
    #             project_gravity,
    #             cmds*self.commands_scale,
    #             (dof_pos - self.default_dof_pos[0:4])*self.obs_scales['dof_pos'],
    #             dof_vel*self.obs_scales["dof_vel"],
    #             actions,
    #         ]
    #     )
    #     slice_obs_buf = slice_obs_buf.unsqueeze(0)
    #     self.obs_buf = torch.cat([self.history_obs_buf, slice_obs_buf], dim=0).view(-1)
        
    #     if self.obs_cfg["history_length"] > 1:
    #         self.history_obs_buf[:-1, :] = self.history_obs_buf[1:, :].clone()  # 移位操作
    #     self.history_obs_buf[-1, :] = slice_obs_buf 
        
    #     target_dof_pos = actions[0:4] * 0.05 + self.default_dof_pos[0:4]
    #     target_dof_vel = actions[4:6] * 1 #env_cfg["wheel_action_scale"]
    #     target_dof_pos = torch.clamp(target_dof_pos, self.dof_pos_lower[0:4],self.dof_pos_upper[0:4])
        
        

def main(args=None):
    robot = robotcontroller()
    
    def start_self_check():
        robot.transition_to(RobotState.SELF_CHECK)
    
    def lock_legs():
        robot.transition_to(RobotState.LOCKING_LEGS)
    
    def emergency_stop():
        robot.transition_to(RobotState.EMERGENCY_STOP)
    
    def start_rlcontrol():
        robot.transition_to(RobotState.RLCONTROL)
        
    command_dict = {
        "self_check": start_self_check,
        "lock": lock_legs,
        "e": emergency_stop,
        "rlcontrol": start_rlcontrol
    }
    def update_loop():
        target_dt = 0.01  
        last_time = time.perf_counter()  
    
        while True:
            current_time = time.perf_counter()  
            elapsed_time = current_time - last_time 
            
            if elapsed_time >= target_dt:
                robot.update()
                last_time = current_time  
            else:
                time_to_wait = target_dt - elapsed_time
                if time_to_wait > 0:
                    time.sleep(time_to_wait)
    
    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()

    while True:
        try:
            cmd = input("CMD: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                print("Exiting program...")
                break
            else:
                print("Unknown command. Available commands:", list(command_dict.keys()) + ["exit"])
        except KeyboardInterrupt:
            update_thread.join()
            print("Program interrupted by user")
            break

if __name__ == '__main__':
    main()