from .modules.pose_capture_node import rclpy, PoseCaptureNode



def main(args=None) -> None:
    rclpy.init()
    node = PoseCaptureNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()