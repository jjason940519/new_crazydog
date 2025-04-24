import serial
import struct
import math
import platform
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler
import time


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.python_version = platform.python_version()[0]
        self.port = self.declare_parameter('port', '/dev/imu').value
        self.baudrate = self.declare_parameter('baudrate', 921600).value
        self.gra_normalization = self.declare_parameter('gra_normalization', True).value

        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'handsfree/mag', 10)

        self.key = 0
        self.buff = {}
        self.angularVelocity = [0., 0., 0.]
        self.acceleration = [0., 0., 0.]
        self.magnetometer = [0., 0., 0.]
        self.angle_degree = [0., 0., 0.]
        self.pub_flag = [True, True]
        self.data_right_count = 0

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        self.find_ttyUSB()
        self.serial_setup()
        self.create_timer(1/300, self.read_serial_data)

    def find_ttyUSB(self):
        self.get_logger().info('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
        self.get_logger().info(f'当前电脑所连接的 USB 串口设备共 {len(posts)} 个: {posts}')

    def serial_setup(self):
        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("串口打开成功...")
            else:
                self.hf_imu.open()
                self.get_logger().info("打开串口成功...")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            exit(0)
        
    def handleSerialData(self, raw_data):
        if self.data_right_count > 200000:
            self.get_logger().error("该设备传输数据错误，退出")
            exit(0)

        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        if self.python_version == '3':
            self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return

        else:
            self.data_right_count = 0
            data_buff = list(self.buff.values())

            if self.buff[2] == 0x2c and self.pub_flag[0]:
                if self.checkSum(data_buff[2:47], data_buff[47:49]):
                    data = self.hex_to_ieee(data_buff[7:47])
                    self.angularVelocity = data[1:4]
                    self.acceleration = data[4:7]
                    self.magnetometer = data[7:10]
                else:
                    self.get_logger().info('校验失败')
                self.pub_flag[0] = False
            elif self.buff[2] == 0x14 and self.pub_flag[1]:
                if self.checkSum(data_buff[2:23], data_buff[23:25]):
                    data = self.hex_to_ieee(data_buff[7:23])
                    self.angle_degree = data[1:4]
                else:
                    self.get_logger().info('校验失败')
                self.pub_flag[1] = False
            else:
                self.get_logger().info(f"该数据处理类没有提供该 {str(self.buff[2])} 的解析 或数据错误")
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            self.pub_flag[0] = self.pub_flag[1] = True
            stamp = self.get_clock().now().to_msg()

            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = "base_link"

            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = "base_link"

            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])

            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]

            self.imu_msg.angular_velocity.x = self.angularVelocity[0]
            self.imu_msg.angular_velocity.y = self.angularVelocity[1]
            self.imu_msg.angular_velocity.z = self.angularVelocity[2]

            acc_k = math.sqrt(self.acceleration[0] ** 2 + self.acceleration[1] ** 2 + self.acceleration[2] ** 2)
            if acc_k == 0:
                acc_k = 1

            if self.gra_normalization:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
            else:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8

            self.mag_msg.magnetic_field.x = self.magnetometer[0]
            self.mag_msg.magnetic_field.y = self.magnetometer[1]
            self.mag_msg.magnetic_field.z = self.magnetometer[2]

            self.imu_pub.publish(self.imu_msg)
            self.mag_pub.publish(self.mag_msg)

    def checkSum(self, list_data, check_data):
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
            if self.python_version == '2':
                ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
            if self.python_version == '3':
                ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data

    def read_serial_data(self):
        try:
            buff_count = self.hf_imu.inWaiting()
        except Exception as e:
            self.get_logger().error(f"exception: {str(e)}")
            self.get_logger().error("imu 失去连接，接触不良，或断线")
            exit(0)
        else:
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(buff_count):
                    self.handleSerialData(buff_data[i])


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
