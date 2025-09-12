import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class PyTalker(Node):
    def __init__(self):
        super().__init__('py_talker')
        self.pub_string = self.create_publisher(String, 'topicname', 10)
        self.pub_int = self.create_publisher(Int32, 'int_topic', 10)
        self.pub_fl = self.create_publisher(Float32, 'float_topic', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.count = 0
        self.get_logger().info('PyTalker up')

    def on_timer(self):
        msg = String()
        num = Int32()
        fl= Float32()
        msg.data = f'Hello World_py: {self.count}'
        num.data = 9000
        fl.data = 0.89
        self.count += 1
        self.get_logger().info(f'Published: str: {msg.data}, num: {num.data}, float: {fl.data}')
        self.pub_string.publish(msg)
        self.pub_fl.publish(fl)
        self.pub_int.publish(num)

def main():
    rclpy.init()
    node = PyTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
