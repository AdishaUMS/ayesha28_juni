import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *



class DXLTestNode(Node):

    def __init__(self) -> None:
        super().__init__('DXLTestNode')

        