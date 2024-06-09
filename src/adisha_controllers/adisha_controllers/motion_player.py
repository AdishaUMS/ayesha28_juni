from .modules.motion_player_node import *



def main(args=None) -> None:
    rclpy.init()
    node = MotionPlayerNode()
    node.dxlSearch()
    node.enableTorque()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()