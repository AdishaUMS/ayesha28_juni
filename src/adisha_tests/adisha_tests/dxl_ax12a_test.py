from .modules.dxl_tests.dxl_ax12a_test_node import *



def main(args=None):
    rclpy.init()
    node = DxlAx12aSyncRWTestNode()
    # node = DxlAx12aRWTestNode()
    node.start()
    
    rclpy.spin(node)
    
    node.stop()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()