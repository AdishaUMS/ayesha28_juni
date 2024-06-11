import yaml
import rclpy
from rclpy.node import Node
import adisha_interfaces.msg as adisha_interfaces



class MotionSetterNode(Node):

    def __init__(self) -> None:
        super().__init__('MotionSetterNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('master_clock', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('motion_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('stabler_kp', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stabler_ki', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stabler_kd', rclpy.Parameter.Type.DOUBLE)

        self.ID             = self.get_parameter('id').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        self.MOTION_PATH    = self.get_parameter('motion_path').value
        self.STABLER_KP     = self.get_parameter('stabler_kp').value
        self.STABLER_KI     = self.get_parameter('stabler_ki').value
        self.STABLER_KD     = self.get_parameter('stabler_kd').value

        self.motion_file = None
        with open(self.MOTION_PATH, 'r') as file:
            self.motion_file = yaml.safe_load(file)

        self.TIMER_PERIOD   = self.motion_file['dt_ms']*1e-3
        self.POINTS_NUM     = self.motion_file['points_num']
        self.ANGLE_TARGET   = []
        self.SPEED_TARGET   = []

        for i in range(self.POINTS_NUM):
            angle_temp = []
            speed_temp = []

            for dxl_id in self.DXL_ID:
                angle_temp.append(self.motion_file[dxl_id]['angle'][i])
                speed_temp.append(self.motion_file[dxl_id]['speed'][i])
            
            self.ANGLE_TARGET.append(angle_temp)
            self.SPEED_TARGET.append(speed_temp)

        self.imu_roll   = 0.0
        self.imu_pitch  = 0.0
        self.imu_yaw    = 0.0
        self.point_idx  = 0
        self.end_info   = True

        self.goal_position_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointPosition,
            topic       = f'{self.ID}/goal_position',
            qos_profile = 1000
        )
        self.goal_position_msg          = adisha_interfaces.JointPosition()
        self.goal_position_msg.dxl_id   = self.DXL_ID.copy()

        self.goal_speed_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_speed',
            qos_profile = 1000
        )
        self.goal_speed_msg         = adisha_interfaces.JointVelocity()
        self.goal_speed_msg.dxl_id  = self.DXL_ID.copy()

        self.inertial_sub  = self.create_subscription(
            msg_type    = adisha_interfaces.Inertial,
            topic       = f'{self.ID}/inertial',
            callback    = self.inertialSubCallback,
            qos_profile = 1000
        )

        self.setter_timer = self.create_timer(
            self.TIMER_PERIOD,
            self.setterTimerCallback
        )



    def inertialSubCallback(self, msg:adisha_interfaces.Inertial) -> None:
        self.imu_roll   = msg.roll
        self.imu_pitch  = msg.pitch
        self.imu_yaw    = msg.yaw


    
    def setterTimerCallback(self) -> None:
        if self.point_idx < self.POINTS_NUM:
            self.goal_speed_msg.val     = self.SPEED_TARGET[self.point_idx].copy()
            self.goal_position_msg.val  = self.ANGLE_TARGET[self.point_idx].copy()

            self.goal_speed_pub.publish(self.goal_speed_msg)
            self.goal_position_pub.publish(self.goal_position_msg)

            self.point_idx += 1
            self.get_logger().info(f'motion set... [{self.point_idx}/{self.POINTS_NUM}]')

        elif self.end_info:
            self.get_logger().info('-----------[ MOTION SET COMPLETED ] -----------')
            self.end_info = False