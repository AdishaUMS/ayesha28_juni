from .modules.button_controller_node import *



def main(args=None) -> None:
    rclpy.init()
    node = ButtonControllerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()