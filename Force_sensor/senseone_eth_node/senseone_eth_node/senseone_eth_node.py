import sys
import struct
import time
import collections
import pysoem
import ctypes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Temperature, Imu
from std_msgs.msg import Header


class BotaSensorNode(Node):

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 512
    time_step = 1.0

    def __init__(self, ifname):
        super().__init__('bota_sensor_node')
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {
            0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)
        }

        self.publisher_wrench_ = self.create_publisher(WrenchStamped, 'Senseone_eth/wrench', 10)
        self.publisher_imu_ = self.create_publisher(Imu, 'Senseone_eth/imu', 10)
        self.publisher_temperature_ = self.create_publisher(Temperature, 'Senseone_eth/temperature', 10)

        self.get_logger().info("Bota Sensor Node initialized")

    def bota_sensor_setup(self, slave_pos):
        slave = self._master.slaves[slave_pos]

        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))  # calibration matrix active
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))  # temperature compensation
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))  # IMU active

        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))  # FIR disable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))  # FAST enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))  # CHOP enable
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))  # Sinc filter size

        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        if sampling_rate > 0:
            self.time_step = 1.0 / float(sampling_rate)

    def run(self):
        try:
            self._master.open(self._ifname)

            if self._master.config_init() > 0:
                for i, slave in enumerate(self._master.slaves):
                    assert slave.man == self.BOTA_VENDOR_ID
                    assert slave.id == self._expected_slave_mapping[i].product_code
                    slave.config_func = self._expected_slave_mapping[i].config_func

                self._master.config_map()

                if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                    raise Exception('Not all slaves reached SAFEOP state')

                self._master.state = pysoem.OP_STATE
                self._master.write_state()

                if self._master.state_check(pysoem.OP_STATE, 50000) != pysoem.OP_STATE:
                    raise Exception('Not all slaves reached OP state')

                start_time = time.time()
                self.get_logger().info("Start recording...")
                x = 0

                while rclpy.ok():
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)

                    sensor_input_as_bytes = self._master.slaves[0].input
                    status = struct.unpack_from('B', sensor_input_as_bytes, 0)[0]
                    warningsErrorsFatals = struct.unpack_from('I', sensor_input_as_bytes, 1)[0]

                    # WrenchStamped
                    wrench_stamped = WrenchStamped()
                    wrench_stamped.header = Header()
                    wrench_stamped.header.stamp = self.get_clock().now().to_msg()
                    wrench_stamped.header.frame_id = "wrench_sensor_frame"  # Change if needed

                    wrench_stamped.wrench.force.x = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    wrench_stamped.wrench.force.y = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    wrench_stamped.wrench.force.z = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    wrench_stamped.wrench.torque.x = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    wrench_stamped.wrench.torque.y = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    wrench_stamped.wrench.torque.z = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]

                    # IMU
                    imu_data = Imu()
                    imu_data.angular_velocity.x = struct.unpack_from('f', sensor_input_as_bytes, 44)[0]
                    imu_data.angular_velocity.y = struct.unpack_from('f', sensor_input_as_bytes, 48)[0]
                    imu_data.angular_velocity.z = struct.unpack_from('f', sensor_input_as_bytes, 52)[0]
                    imu_data.linear_acceleration.x = struct.unpack_from('f', sensor_input_as_bytes, 31)[0]
                    imu_data.linear_acceleration.y = struct.unpack_from('f', sensor_input_as_bytes, 35)[0]
                    imu_data.linear_acceleration.z = struct.unpack_from('f', sensor_input_as_bytes, 39)[0]

                    # Temperature
                    temperature_data = Temperature()
                    temperature_data.temperature = struct.unpack_from('f', sensor_input_as_bytes, 57)[0]

                    # Publish
                    self.publisher_wrench_.publish(wrench_stamped)
                    self.publisher_imu_.publish(imu_data)
                    self.publisher_temperature_.publish(temperature_data)

                    time.sleep(self.time_step)
                    if time.time() - start_time > 1 and x == 0:
                        x = 1
                        self.get_logger().info('Calibration complete')

            else:
                self.get_logger().error('Slaves not found')

        except KeyboardInterrupt:
            self.get_logger().info('Stopped by user')
        finally:
            self._master.state = pysoem.INIT_STATE
            self._master.write_state()
            self._master.close()


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run bota_sensor_node bota_sensor_node <interface_name>")
        return

    interface_name = sys.argv[1]
    node = BotaSensorNode(interface_name)
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
