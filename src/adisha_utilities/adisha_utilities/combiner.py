from .modules.combiner_utils import *



def main(args=None) -> None:
    rclpy.init()
    node = CombinerUtils()

    node.executeCombiner()

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()