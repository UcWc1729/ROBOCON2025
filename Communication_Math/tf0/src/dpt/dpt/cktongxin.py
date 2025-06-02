import rclpy
from rclpy.node import Node
import serial
import time
class SerialSender(Node):
    def __init__(self,param):
        super().__init__('serial_sender')
        self.param=param
        self.get_logger().info('Serial Sender Node initialized')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.start_time=time.time()
        self.timer = self.create_timer(1.0, self.send_parameter)

    def send_parameter(self):    
        if time.time()-self.start_time<10:
            param_s=(str(self.param)).encode()
            self.serial_port.write(param_s)
            self.get_logger().info(f'Sent: {self.param}')
        else:
            self.timer.cancel()
