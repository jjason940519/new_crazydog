import time
import sys
import matplotlib.pyplot as plt
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import * # type: ignorei
import  math


class unitree_communication(object):
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []

    def createMotor(self,motor_number = 0,MAX = 0,MIN = 0,initalposition = 0):
        if motor_number not in [motor.id for motor in self.motors]:
            motor = unitree_motor(motor_number, MAX_degree=MAX, MIN_degree=MIN, inital_position_check=initalposition)
            self.motors.append(motor)
            return motor                              

        else:
            print("Motor {0} already exist".format(motor_number))
            for motor in self.motors:
                if motor.cmd.id == motor_number:
                    return motor

    def inital_all_motor(self):
        for motor in self.motors:
            motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
            motor.cmd.q    = 0.0
            motor.cmd.dq   = 0
            motor.cmd.kp   = 0.0
            motor.cmd.kd   = 0
            motor.cmd.tau  = 0.0
            self.serial.sendRecv(motor.cmd, motor.data)
            time.sleep(0.1)
            if motor.inital_position_max >= motor.data.q and motor.data.q >= motor.inital_position_min:
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = 0.0
                motor.cmd.dq   = 0
                motor.cmd.kp   = 0.0
                motor.cmd.kd   = 0
                motor.cmd.tau  = 0.0
                self.serial.sendRecv(motor.cmd, motor.data)
                time.sleep(0.1)
                motor.inital_position = motor.data.q
                motor.max_position = motor.inital_position + motor.max
                motor.min_position = motor.inital_position + motor.min
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = motor.inital_position
                motor.cmd.dq   = 0
                motor.cmd.kp   = 6.0
                motor.cmd.kd   = 0.1
                motor.cmd.tau  = 0.0
                self.serial.sendRecv(motor.cmd, motor.data)
                # motor.cmd.q    = motor.inital_position
                # motor.cmd.dq   = 0
                # motor.cmd.kp   = 20
                # motor.cmd.kd   = 0
                # motor.cmd.tau  = 0.0
            else:
                print("motor {0} inital motor failed".format(motor.cmd.id))
                print(motor.inital_position_max,motor.data.q,motor.inital_position_min)

    def disableallmotor(self):
        for motor in self.motors:
            motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
            motor.cmd.q    = 0.0
            motor.cmd.dq   = 0
            motor.cmd.kp   = 0.0
            motor.cmd.kd   = 0
            motor.cmd.tau  = 0.0
        for motor in self.motors:
            self.serial.sendRecv(motor.cmd, motor.data)
            time.sleep(0.01)

    def calibrate_all_motor(self):
        for motor in self.motors:
            motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.CALIBRATE)
        for motor in self.motors:
            self.serial.sendRecv(motor.cmd, motor.data)
            time.sleep(6)  

    def position_force_velocity_cmd(self,motor_number=0,torque=0,kp=0,kd=0,position=0,velocity=0):
        for motor in self.motors:
            if motor.id == motor_number:
                if  motor.max_position>=motor.data.q and motor.data.q>=motor.min_position:
                    motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                    motor.cmd.tau = torque
                    motor.cmd.kp = kp
                    motor.cmd.kd = kd
                    motor.cmd.q = position
                    motor.cmd.dq = velocity*queryGearRatio(MotorType.GO_M8010_6)
                    self.serial.sendRecv(motor.cmd, motor.data)
                    time.sleep(0.001)
                else:
                    print("motor {0} out of constrant".format(motor.cmd.id))
                    print(motor.max_position,motor.data.q,motor.min_position)
            else:
                pass

    
class unitree_motor(object):                                                                                  
    def __init__(self,motor_id = 0,MAX_degree = 0,MIN_degree = 0, inital_position_check = 0):
        
        self.id = motor_id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.motorType = MotorType.GO_M8010_6
        self.inital_position_cheak_point = inital_position_check
        self.inital_position_max = inital_position_check + 1
        self.inital_position_min = inital_position_check - 1
        self.inital_position = 0
        self.max_position = 0
        self.min_position = 0 
        self.max = MAX_degree
        self.min = MIN_degree
        self.cmd.id = motor_id

# unitree = unitree_communication('/dev/ttyUSB0')
# MOTOR1 = unitree.createMotor(motor_number = 1,inital_position_cheak = 0.669,MAX_degree=8.475,MIN_degree=-5.364)
# MOTOR2 = unitree.createMotor(motor_number = 2,inital_position_cheak = 3.815,MAX_degree=26.801,MIN_degree=0)
# unitree2 = unitree_communication('/dev/ttyUSB1')
# MOTOR4 = unitree2.createMotor(motor_number = 4,inital_position_cheak = 1.247,MAX_degree=5.364,MIN_degree=-8.475)
# MOTOR5 = unitree2.createMotor(motor_number = 5,inital_position_cheak = 5.046,MAX_degree=0,MIN_degree=-26.801)


def test():
    try:
        unitree.inital_all_motor()
        unitree2.inital_all_motor()
        unitree.position_force_velocity_cmd(motor_number = 1,kp = 1,kd = 0.1,velocity = 1, position = MOTOR1.data.q + 0.7*6.33)
        unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = 1,kd = 0.1,velocity = 1, position = MOTOR4.data.q + 0.7*6.33)
        time.sleep(0.1)
        
    
    except KeyboardInterrupt:
        unitree.disableallmotor()
        unitree2.disableallmotor()


def main():
    
    # 初始化图表
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2,1,sharex=True)
    x_data, angle_data ,dq_data= [], [], []
    line, = ax1.plot(x_data, angle_data)
    line1, = ax2.plot(x_data, angle_data) 

    # 设置图表的初始范围
    ax1.set_xlim(0, 10)
    ax1.set_ylim(-8, 8)
    ax2.set_ylim(-10,10)

    ax1.set_xlabel("time(s)")
    ax1.set_ylabel("rad")
    ax2.set_ylabel("rad/s")

    unitree.inital_all_motor()
    print("inital_positon1:"+str(MOTOR1.inital_position))
    print("inital_positon2:"+str(MOTOR2.inital_position))
    unitree.position_force_cmd(motor_number=1, torque=0, kp=20, kd=0, position= MOTOR1.data.q)

    bottom = 3.5 
    top = -6.38
    angular_velocitgy = 50
    step_size = angular_velocitgy/1000

    _dir = 1

    # 开始动态更新
    start_time = time.time()
    while True:
        # range top~bottom := -6.38~3 -> +offset := 0~9.38 -> 6.38
        while _dir == 1:
            current_time = time.time()-start_time
            x_data.append(current_time)
            current_position = MOTOR2.data.q
            current_position -= step_size
            unitree.position_force_cmd(motor_number=2, torque=0, kp=20, kd=0.16, position=current_position)
            if MOTOR2.data.q <= top:    
                _dir = 0

            angle_data.append(MOTOR2.data.q)
            dq_data.append(MOTOR2.data.dq)
            # 更新数据
            line.set_xdata(x_data)
            line.set_ydata(angle_data)
            line1.set_xdata(x_data)
            line1.set_ydata(dq_data)
            # 更新图表范围
            ax1.set_xlim(max(0, current_time - 10), current_time) 
            ax2.set_xlim(max(0, current_time - 10), current_time)
            # 重绘图形
            fig.canvas.draw()
            fig.canvas.flush_events()

            time.sleep(0.0002)

        while _dir == 0:
            current_time = time.time()-start_time
            x_data.append(current_time)
            current_position = MOTOR2.data.q
            current_position += step_size
            unitree.position_force_cmd(motor_number=2, torque=0, kp=20, kd=0.16, position=current_position)
            if MOTOR2.data.q >= bottom:
                _dir = 1

            angle_data.append(MOTOR2.data.q)
            dq_data.append(MOTOR2.data.dq)  
            # 更新数据
            line.set_xdata(x_data)
            line.set_ydata(angle_data)
            line1.set_xdata(x_data)
            line1.set_ydata(dq_data)
            # 更新图表范围
            ax1.set_xlim(max(0, current_time - 10), current_time) 
            ax2.set_xlim(max(0, current_time - 10), current_time) 
            # 重绘图形
            fig.canvas.draw()
            fig.canvas.flush_events()

            time.sleep(0.0002)

def profile_velocity_control():
    
    bottom = 6
    top = -6.38
    angular_velocity=math.pi/5
  
    plt.ion()
    fig,  (ax1,ax2,ax3) = plt.subplots(3,1, sharex=True)
    x_data, velocity_data, refvelocity_data, position_data , tau_data= [], [], [], [], []
    line1, = ax1.plot(x_data, velocity_data, label="motor_dq")
    line2, = ax1.plot(x_data, refvelocity_data, label="ref_dq")
    line3, = ax2.plot(x_data, position_data, label = "motor_q")
    line4, = ax3.plot(x_data, tau_data, label="motor_tau")

    # 设置图表的初始范围
    ax1.set_xlim(0, 10)
    ax1.set_ylim(-10, 10)
    ax2.set_ylim(-8, 8)
    ax3.set_ylim(-2, 2)

    ax1.set_ylabel("angular_velocity (rad/s)")
    ax1.set_xlabel("time (s)")
    ax2.set_ylabel("angle (rad)")
    ax3.set_ylabel("tau(N/m)")

    ax1.legend()
    ax2.legend()

    unitree.inital_all_motor()
    print("inital_positon1:"+str(MOTOR1.inital_position))
    print("inital_positon2:"+str(MOTOR2.inital_position))
    unitree.position_force_cmd(motor_number=1, torque=0, kp=0.8, kd=0, position= MOTOR1.data.q)

    _dir = 1

    start_time = time.time()
    try:
        while True:
            
            while _dir == 1:

                current_time = time.time()-start_time
                unitree.angular_velocity_cmd(motor_number=2, kd = 0.1, velocity = -angular_velocity)
                x_data.append(current_time)
                velocity_data.append(MOTOR2.data.dq)
                refvelocity_data.append(MOTOR2.cmd.dq)
                position_data.append(MOTOR2.data.q)
                tau_data.append(MOTOR2.data.tau)
                
                # 更新数据
                line1.set_xdata(x_data)
                line1.set_ydata(velocity_data)
                line2.set_xdata(x_data)
                line2.set_ydata(refvelocity_data)
                line3.set_xdata(x_data)
                line3.set_ydata(position_data)
                line4.set_xdata(x_data)
                line4.set_ydata(tau_data)
                # 更新图表范围
                ax1.set_xlim(max(0, current_time - 10), current_time) 
                ax2.set_xlim(max(0, current_time - 10), current_time)
                # 重绘图形
                fig.canvas.draw()
                fig.canvas.flush_events()


                if MOTOR2.data.q <= top:    
                    _dir = 0

                time.sleep(0.0001)

            while _dir == 0:
                
                current_time = time.time()-start_time
                unitree.angular_velocity_cmd(motor_number=2, kd = 0.1, velocity = angular_velocity)
                x_data.append(current_time)
                velocity_data.append(MOTOR2.data.dq*1.6)
                refvelocity_data.append(MOTOR2.cmd.dq)
                position_data.append(MOTOR2.data.q)
                tau_data.append(MOTOR2.data.tau)

                # 更新数据
                line1.set_xdata(x_data)
                line1.set_ydata(velocity_data)
                line2.set_xdata(x_data)
                line2.set_ydata(refvelocity_data)
                line3.set_xdata(x_data)
                line3.set_ydata(position_data)
                line4.set_xdata(x_data)
                line4.set_ydata(tau_data)
                # 更新图表范围
                ax1.set_xlim(max(0, current_time - 10), current_time) 
                ax2.set_xlim(max(0, current_time - 10), current_time)
                # 重绘图形
                fig.canvas.draw()
                fig.canvas.flush_events()

                if MOTOR2.data.q >= bottom:
                    _dir = 1
        
                time.sleep(0.0001)
    except KeyboardInterrupt:
        
        unitree.position_force_cmd(motor_number=2, torque=0, kp=0, kd=0, position= 0)

if __name__ == "__main__":

    profile_velocity_control()
    # main()