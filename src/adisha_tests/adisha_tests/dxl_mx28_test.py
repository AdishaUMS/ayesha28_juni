from .modules.dxl_tests.dxl_mx28_test_node import *



def main(args=None):
    rclpy.init()
    node = DxlMx28SyncRWTestNode()
    # node = DxlMx28RWTestNode()
    node.start()
    
    rclpy.spin(node)
    
    node.stop()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()