import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)  # 每0.1秒发布一次
        self.parent_frame = 'camera_init'
        self.child_frame = 'aft_mapped'

        # 设置变换信息
        self.translation = [3.0, 9.0, 3.0]  # 平移向量
        self.rotation = [0.0, 0.0, 0.0, 1.0]  # 四元数表示的旋转（单位四元数）

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # 设置平移
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]

        # 设置旋转
        t.transform.rotation.x = self.rotation[0]
        t.transform.rotation.y = self.rotation[1]
        t.transform.rotation.z = self.rotation[2]
        t.transform.rotation.w = self.rotation[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform from {self.parent_frame} to {self.child_frame}")

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
