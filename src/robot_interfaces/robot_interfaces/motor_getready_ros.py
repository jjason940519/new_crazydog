import time
import math
import sys
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/utils')
# import unitree_motor_command_thread as um
import threading

import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState


# unitree = um.unitree_communication('/dev/unitree-l')
# MOTOR1 = unitree.createMotor(motor_number = 1,initalposition = 0.669,MAX=8.475,MIN=-5.364)
# MOTOR2 = unitree.createMotor(motor_number = 2,initalposition = 3.815,MAX=26.801,MIN=-1)
# unitree2 = um.unitree_communication('/dev/unitree-r')
# MOTOR4 = unitree2.createMotor(motor_number = 4,initalposition = 1.247+2*math.pi,MAX=5.364,MIN=-8.475)
# MOTOR5 = unitree2.createMotor(motor_number = 5,initalposition = 2.970,MAX=1,MIN=-26.801)


# def disableUnitreeMotor():

#     unitree.disableallmotor()
#     unitree2.disableallmotor()

MOTOR_INIT_POS = [None, 0.669, 3.815, None, 1.247+2*math.pi, 2.970]

class UnitreeManager(Node):

    def __init__(self):
        super().__init__('unitree_manager')
        self.unitree_command_sub = self.create_subscription(
            LowState,
            'unitree_status',
            self.status_callback,
            1)
        self.motor_states = LowState()
        self.motor_cmd_pub = self.create_publisher(LowCommand, 'unitree_command', 1)

    def status_callback(self, msg_list):
        self.motor_states = msg_list.motor_state

class UnitreeControl():
    def __init__(self):
        self.cmd_list = LowCommand()
        rclpy.init()
        self.unitree_manager = UnitreeManager()
        self.unitree_thread = threading.Thread(target=rclpy.spin, args=(self.unitree_manager,), daemon=True)
        self.unitree_thread.start()

    def check_constrain(self, motor_id):
        if motor_id==1 or motor_id==2:
            if self.unitree_manager.motor_states[motor_id].q >= MOTOR_INIT_POS[motor_id]:
                return True
            else: 
                return False
        elif motor_id==4 or motor_id==5:
            if self.unitree_manager.motor_states[motor_id].q <= MOTOR_INIT_POS[motor_id]:
                return True
            else: 
                return False
            
    # def check_initial(self):
    #     for id, motor in enumerate(self.motor_states):
    #         if motor.q

    def set_motor_cmd(self, motor_number, kp, kd, position, torque=0, velocity=0):
        cmd = MotorCommand()
        cmd.q = float(position)
        cmd.dq = float(velocity)
        cmd.tau = float(torque)
        cmd.kp = float(kp)
        cmd.kd = float(kd)
        self.cmd_list.motor_cmd[motor_number] = cmd

    def init_unitree_motor(self):
        if self.check_constrain(1) and self.check_constrain(4):
            while self.check_constrain(1) and self.check_constrain(4):
                self.set_motor_cmd(motor_number=1, kp=2, kd=0.02, position=self.unitree_manager.motor_states[1].q-0.1, torque=0, velocity=0)
                self.set_motor_cmd(motor_number=4, kp=2, kd=0.02, position=self.unitree_manager.motor_states[4].q+0.1, torque=0, velocity=0)
                self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
                time.sleep(0.01)
            time.sleep(0.1)
            self.set_motor_cmd(motor_number=1, kp=6., kd=0.1, position=self.unitree_manager.motor_states[1].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=2, kp=6., kd=0.1, position=self.unitree_manager.motor_states[2].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=4, kp=6., kd=0.1, position=self.unitree_manager.motor_states[4].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=5, kp=6., kd=0.1, position=self.unitree_manager.motor_states[5].q, torque=0, velocity=0)
            self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
            # unitree.inital_check()
            # unitree2.inital_check()
        else:
            print("inital fail")

    def locklegs(self):
        while self.unitree_manager.motor_states[1].q >= MOTOR_INIT_POS[1] + 0.33*6.33 and self.check_constrain(4):            
            self.set_motor_cmd(motor_number=1, kp=2, kd=0.02, position=self.unitree_manager.motor_states[1].q+0.1, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=4, kp=2, kd=0.02, position=self.unitree_manager.motor_states[4].q-0.1, torque=0, velocity= 0)
            self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        for i in range(36):                        
            self.set_motor_cmd(motor_number=1, kp=i, kd=0.12, position=MOTOR_INIT_POS[1]+0.33*6.33)
            self.set_motor_cmd(motor_number=4, kp=i, kd=0.12, position=MOTOR_INIT_POS[4]-0.33*6.33)
            self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        while self.unitree_manager.motor_states[2].q >= MOTOR_INIT_POS[2] + 0.33*6.33*1.6 and self.unitree_manager.motor_states[5].q <= MOTOR_INIT_POS[5]-0.33*6.33*1.6:
            self.set_motor_cmd(motor_number=2, kp=0, kd=0.16, position=0, torque=0, velocity=0.01)
            self.set_motor_cmd(motor_number=5 ,kp=0, kd=0.16, position=0, torque=0, velocity=-0.01)
            self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        for i in range(36):                        
            self.set_motor_cmd(motor_number=2, kp=i, kd=0.15, position=MOTOR_INIT_POS[2]+0.6*6.33*1.6)
            self.set_motor_cmd(motor_number=5, kp=i, kd=0.15, position=MOTOR_INIT_POS[5]-0.6*6.33*1.6)
            self.unitree_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.1)

# def enable():

#     unitree.enableallmotor()
#     unitree2.enableallmotor()

def main():
    unitree_control = UnitreeControl()
    command_dict = {
        # "d": disableUnitreeMotor,
        "i": unitree_control.init_unitree_motor,
        "l": unitree_control.locklegs,
        # "s": enable,
    }
    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                # disableUnitreeMotor()
                break
        except KeyboardInterrupt:
            # disableUnitreeMotor()
            break

if __name__ == '__main__':
    main()