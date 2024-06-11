from .modules.motion_setter_node import *



def main(args=None) -> None:
    rclpy.init()
    node = MotionSetterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()