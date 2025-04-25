import rclpy
import threading
import math
import time
import numpy as np
import yaml
import sys
from pid import PID

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import *

class robotestimater(Node):
    def __init__(self):
        super().__init__('ros_topic_manager')
        
        self.foc_vel_control_dt = 1/500
        
        self.vel_subscriber = self.create_subscription(Twist,'cmd_vel', self.__vel_callback, 1)
        self.foc_data_subscriber = self.create_subscription(Float32MultiArray,'foc_msg',self.__foc_status_callback, 1)
        self.filter_eular = self.create_subscription(Vector3Stamped, 'filter/euler', self.__filter_eular_callback, 1)
        self.imu_acc_subscriber = self.create_subscription(Vector3Stamped, 'imu/acceleration_hr', self.__imu_acc_hr_callback, 1)
        self.imu_ang_vel_subscriber = self.create_subscription(Vector3Stamped, 'imu/angular_velocity_hr', self.__imu_ang_vel_hr_callback, 1)
        self.imu_data_subscriber = self.create_subscription(Imu, 'imu/data', self.__imu_data_callback, 1)
        self.foc_command_publisher = self.create_publisher(Float32MultiArray, 'foc_command', 1)
        self.foc_control_timer = self.create_timer(self.foc_vel_control_dt, self.__foc_control_callback)
            
        #joystick register
        self.joy_linear_vel = 0.
        self.joy_angular_vel = 0.

        # self.wheel_coordinate = [0.033-0.0752, -0.2285]  # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        # self.knee_angle = -2.14

        self.eular_msg = Vector3Stamped()
        self.acc_msg = Vector3Stamped()
        self.ang_vel_msg = Vector3Stamped()
        self.foc_status = Float32MultiArray()
        self.imu_data_msg = Imu()
        
        #kalman filter register
        self.k_x = np.array([[0],[0]])
        self.P_1 = np.array([[0.1, 0],[0, 0.1]])
        
        #foc PI gain
        self.foc_left_pi = PID(Kp=10,Ki=0.5,Kd=0)
        self.foc_right_pi = PID(Kp=10,Ki=0.5,Kd=0)
        
        #foc velocity register
        self.R_vel = 0.0
        self.L_vel = 0.0
        
        self.foc_speed_control = False
        
        # Added variables for euler angle interpolation
        self.prev_euler = None
        self.curr_euler = None
        self.prev_timestamp = None
        self.curr_timestamp = None
        
        self.wheel_radius = 0.07
        
        try:
            with open('unitree_motor.yaml', 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Error: YAML file 'unitree_motor.yaml' not found.")
            return
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")
            return
                
        right_motor_config = config.get('unitree_right', {})
        activate_motors_R = right_motor_config.get('activate_motors', [])
        motors_config_R = right_motor_config.get('motors', {})
        
        left_motor_config = config.get('unitree_left', {})
        activate_motors_L = left_motor_config.get('activate_motors', [])
        motors_config_L = left_motor_config.get('motors', {})
        
        jointstate_config = config.get('jointstate', {})
        motor_origin_pos = jointstate_config.get('motor_origin_pos', {})
        joint_scale = jointstate_config.get('scale', {})
        
        self.scales = np.array(joint_scale, dtype=np.float32)
        self.motor_origin_pos = np.array(motor_origin_pos, dtype=np.float32)
        
        self.unitree_r = unitree_communication('/dev/unitree-r')
        self.unitree_l = unitree_communication('/dev/unitree-l')
        
        self.robotcontroller_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.robotcontroller_thread.start()
        
        for id in activate_motors_R:
            motor_data = motors_config_R.get(id, {})
            origin = motor_data.get('origin')
            scale = motor_data.get('scale')
            max = motor_data.get('max')
            min = motor_data.get('min')
            self.unitree_r.createMotor(motor_number=id, MAX=max, MIN=min, origin_pos=origin, scale=scale)
            
        for id in activate_motors_L:
            motor_data = motors_config_L.get(id, {})
            origin = motor_data.get('origin')
            scale = motor_data.get('scale')
            max = motor_data.get('max')
            min = motor_data.get('min')
            self.unitree_l.createMotor(motor_number=id, MAX=max, MIN=min, origin_pos=origin, scale=scale)
    
        self.num_motors = len(self.unitree_l.motors) + len(self.unitree_r.motors)
        self.unitree_motor_pos = np.zeros((self.num_motors, 1))
        self.unitree_motor_vel = np.zeros((self.num_motors, 1))
        self.unitree_motor_state = np.zeros((self.num_motors, 3))
        self.unitree_cmd = np.zeros((self.num_motors, 5))
        self.scaled_cmd = np.zeros_like(self.unitree_cmd)
        self.unitree_motor_mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.motor_id_to_index = {}
        self.all_motors = self.unitree_l.motors + self.unitree_r.motors # 建立馬達 ID 到索引的映射
        for idx, motor in enumerate(self.all_motors):
            self.motor_id_to_index[motor.id] = idx
        
        self.foc_motor_status = np.zeros((2, 2))
        # self.robotcontroller_thread = threading.Thread(target=rclpy.spin)

        
    def __foc_status_callback(self, msg: Float32MultiArray):
        self.foc_status = msg
        try:
            if self.foc_status.data[0] == 513.:   # motor left
                self.foc_motor_status[0, 0] = -self.foc_status.data[1] #pos
                self.foc_motor_status[0, 1] = -self.foc_status.data[2] * (2 * math.pi / 60) #vel
            elif self.foc_status.data[0] == 514.: # motor right
                self.foc_motor_status[1, 0] = self.foc_status.data[1]
                self.foc_motor_status[1, 1] = self.foc_status.data[2] * (2 * math.pi / 60)
        except Exception as e:
            pass
        
    def __imu_acc_hr_callback(self, msg: Vector3Stamped):
        self.acc_msg = msg
        
    def __imu_ang_vel_hr_callback(self, msg: Vector3Stamped):
        self.ang_vel_msg = msg
        
    def __imu_data_callback(self, msg: Imu):
        self.imu_data_msg = msg

    def __vel_callback(self, msg: Twist):
        # self.joy_linear_vel = msg.linear.x
        self.joy_angular_vel = msg.angular.z
        # Smooth velocity
        alpha = 0.1
        self.joy_linear_vel = msg.linear.x
        
    def __filter_eular_callback(self, msg: Vector3Stamped):
        self.eular_msg = msg
        
        # Store previous and current euler angles for interpolation
        self.prev_euler = self.curr_euler
        self.prev_timestamp = self.curr_timestamp
        
        # Store current euler angles and timestamp
        self.curr_euler = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.curr_timestamp = time.time()  # Use system time for simplicity
        
    def __send_foc_command(self, tau_left, tau_right):
        msg = Float32MultiArray()
        # torque_const = 0.247  # 0.3 N-m/A
        msg.data = [-tau_left, tau_right]
        self.foc_command_publisher.publish(msg)
        
    def __foc_control_callback(self):
        
        if self.foc_speed_control:
            L_torque = self.foc_left_pi.update(target=self.L_vel, now=self.foc_motor_status[0, 1],dt=self.foc_vel_control_dt)
            R_torque = self.foc_right_pi.update(target=self.R_vel, now=self.foc_motor_status[1, 1],dt=self.foc_vel_control_dt)
            self.__send_foc_command(L_torque, R_torque)
        else:
            self.__send_foc_command(0.0, 0.0)
        
    def set_foc_velocity(self, vel_left, vel_right):
        self.R_vel = vel_right
        self.L_vel = vel_left
        
    def __interpolate_euler(self, target_time):
        #Interpolate euler angles at specified time
        if self.prev_euler is None or self.curr_euler is None:
            return None
        
        # Check if target time is within our sample range
        if target_time < self.prev_timestamp or target_time > self.curr_timestamp:
            return None
        
        # Calculate interpolation ratio
        ratio = (target_time - self.prev_timestamp) / (self.curr_timestamp - self.prev_timestamp)
        
        # Linear interpolation of the euler angles
        interpolated = self.prev_euler + ratio * (self.curr_euler - self.prev_euler)
        
        return interpolated
    
    def __get_interpolated_filter_euler(self):
        """Get the most recently interpolated euler angles"""
        if self.prev_euler is None or self.curr_euler is None:
            return self.eular_msg.vector.x, self.eular_msg.vector.y, self.eular_msg.vector.z
        
        current_time = time.time()
        interpolated = self.__interpolate_euler(current_time)
        
        if interpolated is not None:
            return interpolated[0], interpolated[1], interpolated[2]
        else:
            return self.eular_msg.vector.x, self.eular_msg.vector.y, self.eular_msg.vector.z
    
    def get_filter_pitch_orintation(self):
        roll, pitch, yaw= self.__get_interpolated_filter_euler()
        ang_vel_y = self.ang_vel_msg.vector.y
        pitch = (pitch/360)*2*math.pi
        return pitch, ang_vel_y
    
    def get_filter_yaw_orintation(self):
        roll, pitch, yaw= self.__get_interpolated_filter_euler()
        ang_vel_z = self.ang_vel_msg.vector.z
        yaw = (yaw/360)*2*math.pi
        return yaw, ang_vel_z
    
    def get_filter_roll_orintation(self):
        roll, pitch, yaw= self.__get_interpolated_filter_euler()
        ang_vel_x = self.ang_vel_msg.vector.x
        roll = (roll/360)*2*math.pi
        return roll, ang_vel_x
    
    def get_all_angular_velocity(self):
        ang_vel_x = self.ang_vel_msg.vector.x 
        ang_vel_y = self.ang_vel_msg.vector.y 
        ang_vel_z = self.ang_vel_msg.vectot.z
        return ang_vel_x,ang_vel_y,ang_vel_z
    
    def get_gravity_orientation(self):
        qw = self.imu_data_msg.orientation.w
        qx = self.imu_data_msg.orientation.x
        qy = self.imu_data_msg.orientation.y
        qz = self.imu_data_msg.orientation.z

        gravity_orientation = np.zeros(3)

        gravity_orientation[0] = 2 * (qx * qz - qw * qy)
        gravity_orientation[1] = -2 * (qy * qz + qw * qx)
        gravity_orientation[2] = -1 - 2 * (qx**2 + qy**2)

        return gravity_orientation

    def get_joy_vel(self):
        return self.joy_linear_vel, self.joy_angular_vel
    
    def __kalman_filter_data_fusion(self, dt, x, P): 
        sigma_v = 0.25 #速度噪聲方差
        sigma_a = 0.3 #加速度噪聲方差
        sigma_process = 1 #過程噪聲方差
        F = np.array([[1, dt],[0, 1]]) #狀態轉移矩陣
        H = np.eye(2) #量測矩陣
        Q = np.array([[sigma_v, 0],[0, sigma_a]]) #量測協方差矩陣
        R = np.array([[sigma_process, 0],[0, sigma_process]]) #過程協方差矩陣
        # gamma = np.array([[0.5*dt**2],[dt]])
        #預測步驟
        x_hat = F @ x #先驗估計
        P = F @ P @ F.T + Q #先驗誤差協方差
        #校正步驟
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R) #get kalman gain
        x = x_hat + K @ (x - H @ x_hat) #後驗估計
        P = (np.eye(2) - K @ H) @ P #更新誤差協方差
        return x, P

    def get_linear_vel_x_with_kalman(self, dt):
        
        self.k_x[0, 0] = (self.foc_motor_status[0, 1] * self.wheel_radius + self.foc_motor_status[1, 1] * self.wheel_radius)/2
        self.k_x[1, 0] = self.acc_msg.vector.x
        self.k_x,self.P_1 = self.__kalman_filter_data_fusion(dt,self.k_x,self.P_1)
        
        return self.k_x[0, 0]

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
    
    def set_motor_cmd(self, motor_id, kp=0, kd=0, position=0, torque=0, velocity=0):
        
        if motor_id not in self.motor_id_to_index:
            print(f"Error: Motor ID {motor_id} not found.")
            return False
        idx = self.motor_id_to_index[motor_id]
        torque = max(-5, min(torque, 5))
        self.unitree_cmd[idx, 0] = float(torque)
        self.unitree_cmd[idx, 1] = float(kp)
        self.unitree_cmd[idx, 2] = float(kd)
        self.unitree_cmd[idx, 3] = float(position)
        self.unitree_cmd[idx, 4] = float(velocity)
    
    def unitree_sendrecv(self, free_joint_mode = False):
        
        if free_joint_mode == True:
            self.scaled_cmd[:, 0] = 0
            self.scaled_cmd[:, 1] = 0
            self.scaled_cmd[:, 2] = 0
            self.scaled_cmd[:, 3] = 0
            self.scaled_cmd[:, 4] = 0
        else:
            self.scaled_cmd[:, 0] = self.unitree_cmd[:, 0] / self.scales  # tau
            self.scaled_cmd[:, 1] = self.unitree_cmd[:, 1]  # kp (不縮放)
            self.scaled_cmd[:, 2] = self.unitree_cmd[:, 2]  # kd (不縮放)
            self.scaled_cmd[:, 3] = self.unitree_cmd[:, 3] * self.scales + self.motor_origin_pos # q
            self.scaled_cmd[:, 4] = self.unitree_cmd[:, 4] * self.scales  # dq

        for i, motor in enumerate(self.all_motors):
            motor.cmd.mode = self.unitree_motor_mode
            motor.cmd.tau = self.scaled_cmd[i, 0]
            motor.cmd.kp = self.scaled_cmd[i, 1]
            motor.cmd.kd = self.scaled_cmd[i, 2]
            motor.cmd.q = self.scaled_cmd[i, 3]
            motor.cmd.dq = self.scaled_cmd[i, 4]

        feedback_l, error_id_l = self.unitree_l.motor_sendRecv()
        feedback_r, error_id_r = self.unitree_r.motor_sendRecv()
        
        if feedback_l == False:
            self.error_detected = True
            self.error_message = f"Left motor communication failed, error IDs: {error_id_l}"
            return None 
        
        if feedback_r == False:
            self.error_detected = True
            self.error_message = f"Right motor communication failed, error IDs: {error_id_r}"
            return None
        
        # 批量處理返回數據
        self.unitree_motor_pos = np.array([motor.data.q for motor in self.all_motors], dtype=np.float32)
        self.unitree_motor_vel = np.array([motor.data.dq for motor in self.all_motors], dtype=np.float32)
        self.unitree_motor_torque = np.array([motor.data.tau for motor in self.all_motors], dtype=np.float32)
        
        scaled_positions = (self.unitree_motor_pos - self.motor_origin_pos) / self.scales
        scaled_velocities = self.unitree_motor_vel / self.scales
        scaled_torque = self.unitree_motor_torque / self.scales  
        self.unitree_motor_state = np.stack([scaled_positions, scaled_velocities, scaled_torque], axis=0)
        
    def get_joint_torque(self,motor_id):
        
        if motor_id not in self.motor_id_to_index:
            print(f"Error: Motor ID {motor_id} not found.")
            return False
        idx = self.motor_id_to_index[motor_id]
        return self.unitree_motor_state[2, idx]
        
    def get_joint_pos(self,motor_id):
        
        if motor_id not in self.motor_id_to_index:
            print(f"Error: Motor ID {motor_id} not found.")
            return False
        idx = self.motor_id_to_index[motor_id]
        return self.unitree_motor_state[0, idx]
    
    def get_joint_vel(self,motor_id):
        
        if motor_id not in self.motor_id_to_index:
            print(f"Error: Motor ID {motor_id} not found.")
            return False
        idx = self.motor_id_to_index[motor_id]
        return self.unitree_motor_state[1, idx]

class unitree_communication(object):
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []
        # self.runing_flag = False

    def createMotor(self, motor_number=None, MAX=None, MIN=None, origin_pos=None, scale=None):
        if motor_number not in [motor.id for motor in self.motors]:
            motor = unitree_motor(motor_number, MAX_degree=MAX, MIN_degree=MIN, origin_pos=origin_pos, scale=scale)
            self.motors.append(motor)
            return motor                              

        else:
            print("Motor {0} already exist".format(motor_number))
            for motor in self.motors:
                if motor.cmd.id == motor_number:
                    return motor
            
    def motor_sendRecv(self):
        success = True
        id = []
        for motor in self.motors:
            if motor.motor_check == False:
                
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = 0
                motor.cmd.dq   = 0
                motor.cmd.kp   = 0
                motor.cmd.kd   = 0
                motor.cmd.tau  = 0
                self.serial.sendRecv(motor.cmd, motor.data)
                                     
                if motor.min < (motor.data.q-motor.origin)/motor.scale < motor.max:
                    motor.motor_check = True
                else:
                    print("motor {0} out off constrant".format(motor.cmd.id))
                    motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                    motor.cmd.q    = 0
                    motor.cmd.dq   = 0
                    motor.cmd.kp   = 0
                    motor.cmd.kd   = 0.1
                    motor.cmd.tau  = 0
                    self.serial.sendRecv(motor.cmd, motor.data)
                    print(motor.max, (motor.data.q-motor.origin)/motor.scale, motor.min)
            else:
                if motor.min < (motor.data.q-motor.origin)/motor.scale < motor.max:
                    self.serial.sendRecv(motor.cmd, motor.data)
                else:
                    print("motor {0} out off constrant".format(motor.cmd.id))
                    motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                    motor.cmd.q    = 0
                    motor.cmd.dq   = 0
                    motor.cmd.kp   = 0
                    motor.cmd.kd   = 0.1
                    motor.cmd.tau  = 0
                    self.serial.sendRecv(motor.cmd, motor.data)
                    print(motor.max, (motor.data.q-motor.origin)/motor.scale, motor.min)
                    motor.motor_check = False
                
                    
        return success, id

class unitree_motor(object):                                                                                  
    def __init__(self, motor_id=None,MAX_degree=None,MIN_degree=None, origin_pos=None, scale=None):
        self.id = motor_id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.motorType = MotorType.GO_M8010_6
        self.max = MAX_degree
        self.min = MIN_degree
        self.origin = origin_pos
        self.scale = scale
        self.cmd.id = motor_id
        self.motor_check = False

        # print(f'constrain: id {self.id}, max {self.max}, min {self.min}')
