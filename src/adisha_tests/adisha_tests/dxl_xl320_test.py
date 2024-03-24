from .modules.dxl_tests.dxl_xl320_test_node import *



def main(args=None):
    rclpy.init()
    node = DxlXl320SyncRWTestNode()
    # node = DxlXl320RWTestNode()
    node.start()
    
    rclpy.spin(node)
    
    node.stop()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()