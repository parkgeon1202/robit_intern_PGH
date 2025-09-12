import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32 ,Float32

class PyListener(Node):
    def __init__(self):
        super().__init__('py_listener')
        self.sub_str = self.create_subscription(String, 'topicname', self.on_msg, 10)
        self.sub_int = self.create_subscription(Int32, 'int_topic', self.on_msg, 10)    
        self.sub_float = self.create_subscription(Float32, 'float_topic', self.on_msg, 10)  
        self.get_logger().info('PyListener up')

    def on_msg(self, msg):
        self.get_logger().info("{0}, ".format(msg.data))



def main():
    rclpy.init()
    node = PyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
