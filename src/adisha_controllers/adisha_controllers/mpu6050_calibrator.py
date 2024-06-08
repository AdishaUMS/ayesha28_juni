from .modules.mpu6050_controller_node import *


def main(args=None) -> None:
    rclpy.init()
    node = MPU6050ControllerNode()

    node.calibrateGyro()

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()