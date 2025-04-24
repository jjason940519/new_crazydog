import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import can


class focCommandSubscriber(Node):

    def __init__(self):
        super().__init__('foc_command_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'foc_command',
            self.listener_callback,
            1)

        can_interface = 'can0'
        self.bus = can.interface.Bus(channel=can_interface, interface='socketcan')

    def listener_callback(self, msg):
        motor1_current = max(-10, min(10, msg.data[0]))     # constrain -20~20
        motor2_current = max(-10, min(10, msg.data[1]))     # constrain -20~20
        motor1_cmd = int(motor1_current*16384/20)
        motor2_cmd = int(motor2_current*16384/20)
        print(motor1_cmd, motor2_cmd)
        motor1_highByte, motor1_lowByte = self.int_to_high_low_bytes(motor1_cmd)
        motor2_highByte, motor2_lowByte = self.int_to_high_low_bytes(motor2_cmd)

        self.canMsg = can.Message(
            arbitration_id=0x200,
            data=[motor1_highByte, 
                  motor1_lowByte, 
                  motor2_highByte, 
                  motor2_lowByte,
                  0x00, 0x00, 0x00, 0x00],
            is_extended_id=False  # Use True for extended ID (29-bit)
        )

        try:
            # Send the message
            print('sent message', self.canMsg)
            self.bus.send(self.canMsg)
            
        except can.CanError:
            self.get_logger().error("CAN failed to send message")

    def int_to_high_low_bytes(self, value):
        high_byte = (value >> 8) & 0xFF  # Extract the high byte
        low_byte = value & 0xFF          # Extract the low byte
        
        return high_byte, low_byte
    
    # def twos_complement_16bit(self, value):
    #     if value & (1 << 15):  # Check if the sign bit is set (16th bit)
    #         value = value - (1 << 16)  # Compute the two's complement
    #     return value



def main(args=None):
    rclpy.init(args=args)

    command_subscriber = focCommandSubscriber()

    rclpy.spin(command_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()