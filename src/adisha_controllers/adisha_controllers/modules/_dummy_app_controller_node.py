import numpy as np
import rclpy
from rclpy.node import Node
import adisha_interfaces.msg as adisha_interfaces



class DummyAppControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('DummyAppControllerNode')
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

        self.present_position   = [0 for __ in range(self.DXL_NUM)]
        self.present_speed   = [0 for __ in range(self.DXL_NUM)]
        self.present_torque     = [0 for __ in range(self.DXL_NUM)] 

        self.goal_torque_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointTorque,
            topic       = f'{self.ID}/goal_torque',
            callback    = self.goalTorqueSubCallback,
            qos_profile = 10
        )

        self.goal_position_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointPosition,
            topic       = f'{self.ID}/goal_position',
            callback    = self.goalPositionSubCallback,
            qos_profile = 10
        )

        self.goal_speed_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_speed',
            callback    = self.goalSpeedSubCallback,
            qos_profile = 10
        )

        self.joint_sensor_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointSensor,
            topic       = f'{self.ID}/joint_sensor',
            qos_profile = 10
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

        self.present_speed_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/present_speed',
            qos_profile = 10
        )

        self.pub_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.pubTimerCallback
        )


    
    def goalPositionSubCallback(self, msg:adisha_interfaces.JointPosition) -> None:
        for i in range(self.DXL_NUM):
            self.present_position[i] = msg.val[i]

        self.get_logger().info(f'[{self.ID}/goal_position]: position received')



    def goalSpeedSubCallback(self, msg:adisha_interfaces.JointVelocity) -> None:
        for i in range(self.DXL_NUM):
            self.present_speed[i] = msg.val[i]

        self.get_logger().info(f'[{self.ID}/goal_speed]: speed received')



    def goalTorqueSubCallback(self, msg:adisha_interfaces.JointTorque) -> None:
        for i in range(self.DXL_NUM):
            self.present_torque[i] = msg.val[i]

        self.get_logger().info(f'[{self.ID}/goal_torque]: torque received')



    def pubTimerCallback(self) -> None:
        for i in range(self.DXL_NUM):
            if self.present_torque[i] == 0:
                self.present_position[i]    = np.random.randint(0, 4096)
                self.present_speed[i]       = np.random.randint(0, 4096)

        present_position_msg    = adisha_interfaces.JointPosition()
        present_speed_msg       = adisha_interfaces.JointVelocity()
        present_torque_msg      = adisha_interfaces.JointTorque()
        joint_sensor_msg        = adisha_interfaces.JointSensor()

        present_position_msg.val        = self.present_position[:]
        present_speed_msg.val           = self.present_speed[:]
        present_torque_msg.val          = self.present_torque[:]
        joint_sensor_msg.load           = [(2.0*np.random.rand() - 1.0)*3.0 for __ in range(self.DXL_NUM)]
        joint_sensor_msg.voltage        = [2.0*np.random.rand() + 13.0 for __ in range(self.DXL_NUM)]
        joint_sensor_msg.temperature    = [np.random.rand()*100. for __ in range(self.DXL_NUM)]

        self.present_position_pub.publish(present_position_msg)
        self.present_speed_pub.publish(present_speed_msg)
        self.present_torque_pub.publish(present_torque_msg)  
        self.joint_sensor_pub.publish(joint_sensor_msg)