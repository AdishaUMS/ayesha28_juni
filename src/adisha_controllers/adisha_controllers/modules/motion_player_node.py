import os
import yaml
import time
import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *



class MotionPlayerNode(Node):

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
        self.declare_parameter('pose_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('motion_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('motions', rclpy.Parameter.Type.STRING_ARRAY)

        self.ID             = self.get_parameter('id').value
        self.DXL_BAUDRATE   = self.get_parameter('dxl_baudrate').value
        self.DXL_U2D2_PORT  = self.get_parameter('dxl_u2d2_port').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
        self.JOINT_NAME     = self.get_parameter('joint_name').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        self.POSE_PATH      = self.get_parameter('pose_path').value
        self.MOTION_PATH    = self.get_parameter('motion_path').value
        self.MOTIONS        = self.get_parameter('motions').value
        
        self.xl320_id_list  = []
        self.xl320_id_set   = set(())
        self.xl320_name     = dict(())

        self.ax12a_id_list  = []
        self.ax12a_id_set   = set(())
        self.ax12a_name     = dict(())

        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())

        self.pose_num       = 0
        self.pose_list      = []
        self.delay_list     = []
        self.duration_list  = []
        self.speed_list     = []

        self.write_torque   = False
        self.write_position = False
        self.write_speed    = False

        self.goal_torque    = []
        self.goal_position  = []
        self.goal_speed     = []

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


    
    def dxlSearch(self) -> None:
        for i in range(self.DXL_NUM):
            dxl_id      = self.DXL_ID[i]
            dxl_type    = self.DXL_TYPE[i]
            joint_name  = self.JOINT_NAME[i]

            if dxl_type == 'XL320':
                self.xl320_id_list.append(dxl_id)
                self.xl320_id_set.add(dxl_id)
                self.xl320_name.update({dxl_id: joint_name})

            elif dxl_type == 'AX12A':
                self.ax12a_id_list.append(dxl_id)
                self.ax12a_id_set.add(dxl_id)
                self.ax12a_name.update({dxl_id: joint_name})

            elif dxl_type == 'MX28':
                self.mx28_id_list.append(dxl_id)
                self.mx28_id_set.add(dxl_id)
                self.mx28_name.update({dxl_id: joint_name})

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_MODEL_NUMBER_ADDR,
                    size    = DXL_XL320_MODEL_NUMBER_SIZE,
                    dxl_id  = self.DXL_ID[i]
                )

                if res >= DXL_OK:
                    self.get_logger().info(f'[ID:{self.DXL_ID[i]} ({self.xl320_name[self.DXL_ID[i]]})]: Connection established.')

                else:
                    self.get_logger().error(f'[ID:{self.DXL_ID[i]} ({self.xl320_name[self.DXL_ID[i]]})]: Failed to connect.')
                    self.get_logger().info('Make sure all the servo are connected properly. Terminating.')
                    
                    self.destroy_node()
                    rclpy.shutdown()
                    quit()

            elif self.DXL_ID[i] in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_MODEL_NUMBER_ADDR,
                    size    = DXL_AX12A_MODEL_NUMBER_SIZE,
                    dxl_id  = self.DXL_ID[i]
                )

                if res >= DXL_OK:
                    self.get_logger().info(f'[ID:{self.DXL_ID[i]} ({self.ax12a_name[self.DXL_ID[i]]})]: Connection established.')

                else:
                    self.get_logger().error(f'[ID:{self.DXL_ID[i]} ({self.ax12a_name[self.DXL_ID[i]]})]: Failed to connect.')
                    self.get_logger().info('Make sure all the servo are connected properly. Terminating.')
                    
                    self.destroy_node()
                    rclpy.shutdown()
                    quit()

            elif self.DXL_ID[i] in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_MODEL_NUMBER_ADDR,
                    size    = DXL_MX28_MODEL_NUMBER_SIZE,
                    dxl_id  = self.DXL_ID[i]
                )

                if res >= DXL_OK:
                    self.get_logger().info(f'[ID:{self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Connection established.')

                else:
                    self.get_logger().error(f'[ID:{self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Failed to connect.')
                    self.get_logger().info('Make sure all the servo are connected properly. Terminating.')
                    
                    self.destroy_node()
                    rclpy.shutdown()
                    quit()



    def writeTorque(self) -> None:
        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.xl320_id_set:
                res = self.dxl_controller2.write(
                    address = DXL_XL320_TORQUE_ENABLE_ADDR,
                    size    = DXL_XL320_TORQUE_ENABLE_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_torque[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.xl320_name[self.DXL_ID[i]]})]: Write torque failed')

            elif self.DXL_ID[i] in self.ax12a_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_AX12A_TORQUE_ENABLE_ADDR,
                    size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_torque[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.ax12a_name[self.DXL_ID[i]]})]: Write torque failed')

            elif self.DXL_ID[i] in self.mx28_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_MX28_TORQUE_ENABLE_ADDR,
                    size    = DXL_MX28_TORQUE_ENABLE_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_torque[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Write torque failed')


    
    def writeGoalPosition(self) -> None:
        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.xl320_id_set:
                res = self.dxl_controller2.write(
                    address = DXL_XL320_GOAL_POSITION_ADDR,
                    size    = DXL_XL320_GOAL_POSITION_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_position[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.xl320_name[self.DXL_ID[i]]})]: Write position failed')

            elif self.DXL_ID[i] in self.ax12a_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_AX12A_GOAL_POSITION_ADDR,
                    size    = DXL_AX12A_GOAL_POSITION_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_position[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.ax12a_name[self.DXL_ID[i]]})]: Write position failed')

            elif self.DXL_ID[i] in self.mx28_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_MX28_GOAL_POSITION_ADDR,
                    size    = DXL_MX28_GOAL_POSITION_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_position[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Write position failed')



    def writeGoalSpeed(self) -> None:
        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.xl320_id_set:
                res = self.dxl_controller2.write(
                    address = DXL_XL320_GOAL_SPEED_ADDR,
                    size    = DXL_XL320_GOAL_SPEED_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_speed[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.xl320_name[self.DXL_ID[i]]})]: Write speed failed')

            elif self.DXL_ID[i] in self.ax12a_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_AX12A_GOAL_SPEED_ADDR,
                    size    = DXL_AX12A_GOAL_SPEED_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_speed[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.ax12a_name[self.DXL_ID[i]]})]: Write speed failed')

            elif self.DXL_ID[i] in self.mx28_id_set:
                res = self.dxl_controller1.write(
                    address = DXL_MX28_GOAL_SPEED_ADDR,
                    size    = DXL_MX28_GOAL_SPEED_SIZE,
                    dxl_id  = self.DXL_ID[i],
                    param   = self.goal_speed[i]
                )

                if res < DXL_OK:
                    self.get_logger().error(f'[ID: {self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Write speed failed')



    def loadMotions(self) -> None:
        XL320_DEG_PER_BIT   = 0.29
        XL320_DPS_PER_BIT   = 0.111*6.0
        XL320_DPS_MIN       = 1
        XL320_DPS_MAX       = 1023
        AX12A_DEG_PER_BIT   = 0.29
        AX12A_DPS_PER_BIT   = 0.111*6.0
        AX12A_DPS_MIN       = 1
        AX12A_DPS_MAX       = 1023
        MX28_DEG_PER_BIT    = 0.088
        MX28_DPS_PER_BIT    = 0.114*6.0
        MX28_DPS_MIN        = 1
        MX28_DPS_MAX        = 1023

        for motion in self.MOTIONS:
            self.get_logger().info(f'Loading motion: {motion}.yaml...')
            motion_path = os.path.join(self.MOTION_PATH, f'{motion}.yaml')

            with open(motion_path, 'r') as file:
                motion_yaml = yaml.safe_load(file)
            
            for val in motion_yaml['val']:
                pose_path = os.path.join(self.POSE_PATH, val[0])

                with open(pose_path, 'r') as file:
                    pose_yaml = yaml.safe_load(file)

                self.pose_list.append(pose_yaml['val'])
                self.delay_list.append(val[1])
                self.duration_list.append(val[2])
                self.pose_num += 1

        for i in range(self.pose_num):
            self.get_logger().info(f'Configuring speed [{i + 1}/{self.pose_num}]')

            if i == 0:
                self.speed_list.append([450 for __ in range(self.DXL_NUM)])
                continue

            temp_speed_list = []

            for j in range(self.DXL_NUM):
                if self.DXL_ID[j] in self.xl320_id_set:
                    speed = int(round(abs(self.pose_list[i][j] - self.pose_list[i - 1][j])*(XL320_DEG_PER_BIT*1000./self.duration_list[i])/XL320_DPS_PER_BIT))
                    if speed > XL320_DPS_MAX:   speed = XL320_DPS_MAX
                    elif speed < XL320_DPS_MIN: speed = XL320_DPS_MIN
                    temp_speed_list.append(speed)

                elif self.DXL_ID[j] in self.ax12a_id_set:
                    speed = int(round(abs(self.pose_list[i][j] - self.pose_list[i - 1][j])*(AX12A_DEG_PER_BIT*1000./self.duration_list[i])/AX12A_DPS_PER_BIT))
                    if speed > AX12A_DPS_MAX:   speed = AX12A_DPS_MAX
                    elif speed < AX12A_DPS_MIN: speed = AX12A_DPS_MIN
                    temp_speed_list.append(speed)

                elif self.DXL_ID[j] in self.mx28_id_set:
                    speed = int(round(abs(self.pose_list[i][j] - self.pose_list[i - 1][j])*(MX28_DEG_PER_BIT*1000./self.duration_list[i])/MX28_DPS_PER_BIT))
                    if speed > MX28_DPS_MAX:    speed = MX28_DPS_MAX
                    elif speed < MX28_DPS_MIN:  speed = MX28_DPS_MIN
                    temp_speed_list.append(speed)

            self.speed_list.append(temp_speed_list)



    def playMotion(self) -> None:
        TIME_TOLERANCE_MS = 20

        for i in range(self.pose_num):
            time.sleep(float(self.delay_list[i])/1000.)

            self.goal_speed     = self.speed_list[i][:]
            self.goal_position  = self.pose_list[i][:]
            self.writeGoalSpeed()
            self.writeGoalPosition()

            time.sleep(float(self.duration_list[i])/1000. + TIME_TOLERANCE_MS)