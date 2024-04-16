import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *
import adisha_interfaces.msg as adisha_interfaces



class AppControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('AppControllerNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_baudrate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_u2d2_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('dxl_type', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joint_name', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('master_clock', rclpy.Parameter.Type.DOUBLE)

        self.ID             = self.get_parameter('id').value
        self.DXL_BAUDRATE   = self.get_parameter('dxl_baudrate').value
        self.DXL_U2D2_PORT  = self.get_parameter('dxl_u2d2_port').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
        self.JOINT_NAME     = self.get_parameter('joint_name').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        
        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())
        self.mx64_id_list   = []
        self.mx64_id_set    = set(())
        self.mx64_name      = dict(())

        self.write_torque   = False
        self.write_position = False
        self.write_velocity = False

        self.goal_torque    = []
        self.goal_position  = []
        self.goal_velocity  = []

        self.present_torque         = []
        self.present_position       = []
        self.present_velocity       = []
        self.present_load           = []
        self.present_voltage        = []
        self.present_temperature    = []

        self.porthandler    = dxl.PortHandler(self.DXL_U2D2_PORT)
        self.packethandler1 = dxl.PacketHandler(1.0)
        self.packethandler2 = dxl.PacketHandler(2.0)

        try:
            self.porthandler.openPort()
            self.get_logger().info(f'Port {self.DXL_U2D2_PORT} opened successfully.')

        except:
            self.get_logger().error(f'Failed to open port on {self.DXL_U2D2_PORT}')
            self.get_logger().error(f'Try "sudo chmod a+rw {self.DXL_U2D2_PORT}"')
            quit()

        try:
            self.porthandler.setBaudRate(self.DXL_BAUDRATE)
            self.get_logger().info(f'Baudrate set to {self.DXL_BAUDRATE}')

        except:
            self.get_logger().error(f'Failed to set baudrate to {self.DXL_BAUDRATE}')
            quit()

        self.dxl_controller1 = DXLController(
            self.porthandler, 
            self.packethandler1
        )

        self.dxl_controller2 = DXLController(
            self.porthandler,
            self.packethandler2
        )

        self.goal_torque_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointTorque,
            topic       = f'{self.ID}/goal_torque',
            callback    = self.goalTorqueSubCallback,
            qos_profile = 1000
        )

        self.goal_position_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointPosition,
            topic       = f'{self.ID}/goal_position',
            callback    = self.goalPositionSubCallback,
            qos_profile = 1000
        )

        self.goal_velocity_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_velocity',
            callback    = self.goalVelocitySubCallback,
            qos_profile = 1000
        )

        self.present_torque_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointTorque,
            topic       = f'{self.ID}/present_torque',
            qos_profile = 10
        )

        self.present_position_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointPosition,
            topic       = f'{self.ID}/present_position',
            qos_profile = 10
        )

        self.present_velocity_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/present_velocity',
            qos_profile = 10
        )

        self.joint_sensor_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointSensor,
            topic       = f'{self.ID}/joint_sensor',
            qos_profile = 10
        )

        self.controller_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.controllerTimerCallback
        )



    def dxlSearch(self) -> None:
        pass
            


    def readTorque(self) -> list:
        pass



    def readPresentPosition(self) -> list:
        pass



    def readPresentVelocity(self) -> list:
        pass



    def readPresentLoad(self) -> list:
        pass



    def readPresentInputVoltage(self) -> list:
        pass



    def readPresentTemperature(self) -> list:
        pass



    def writeTorque(self) -> None:
        pass



    def writeGoalPosition(self) -> None:
        pass



    def writeGoalVelocity(self) -> None:
        pass



    def goalTorqueSubCallback(self, msg:adisha_interfaces.JointTorque) -> None:
        pass



    def goalPositionSubCallback(self, msg:adisha_interfaces.JointPosition) -> None:
        pass



    def goalVelocitySubCallback(self, msg:adisha_interfaces.JointVelocity) -> None:
        pass



    def controllerTimerCallback(self) -> None:
        pass