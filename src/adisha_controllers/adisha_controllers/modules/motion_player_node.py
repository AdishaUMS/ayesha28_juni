import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *
import adisha_interfaces.msg as adisha_interfaces
import threading



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
        self.declare_parameter('tracker_kp', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tracker_ki', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tracker_kd', rclpy.Parameter.Type.DOUBLE)

        self.ID             = self.get_parameter('id').value
        self.DXL_BAUDRATE   = self.get_parameter('dxl_baudrate').value
        self.DXL_U2D2_PORT  = self.get_parameter('dxl_u2d2_port').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
        self.JOINT_NAME     = self.get_parameter('joint_name').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        self.TRACKER_KP     = self.get_parameter('tracker_kp').value
        self.TRACKER_KI     = self.get_parameter('tracker_ki').value
        self.TRACKER_KD     = self.get_parameter('tracker_kd').value

        self.xl320_id_list  = []
        self.xl320_id_set   = set(())
        self.xl320_name     = dict(())

        self.ax12a_id_list  = []
        self.ax12a_id_set   = set(())
        self.ax12a_name     = dict(())

        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())

        self.present_position   = dict(())
        self.goal_position      = dict(())
        self.goal_speed         = dict(())

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

        self.dxl_write_mutex = threading.Lock()

        self.goal_position_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointPosition,
            topic       = f'{self.ID}/goal_position',
            callback    = self.goalPositionSubCallback,
            qos_profile = 1000
        )

        self.goal_speed_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_speed',
            callback    = self.goalSpeedSubCallback,
            qos_profile = 1000
        )


    
    def dxlSearch(self) -> None:
        for i in range(self.DXL_NUM):
            dxl_id      = self.DXL_ID[i]
            dxl_type    = self.DXL_TYPE[i]
            joint_name  = self.JOINT_NAME[i]

            self.present_position.update({dxl_id: 0})
            self.goal_position.update({dxl_id: 0})
            self.goal_speed.update({dxl_id: 200})

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



    def enableTorque(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_TORQUE_ENABLE_ADDR,
            size    = DXL_XL320_TORQUE_ENABLE_SIZE,
            dxl_id  = self.xl320_id_list,
            params   = [[1] for __ in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_TORQUE_ENABLE_ADDR,
            size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
            dxl_id  = self.ax12a_id_list,
            params   = [[1] for __ in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            params   = [[1] for __ in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write torque failed')



    def writeGoalPosition(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_GOAL_POSITION_ADDR,
            size    = DXL_XL320_GOAL_POSITION_SIZE,
            dxl_id  = self.xl320_id_list,
            params  = [self.dxl_controller2.convert2ByteToDxl(self.goal_position[dxl_id]) for dxl_id in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write position failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_GOAL_POSITION_ADDR,
            size    = DXL_AX12A_GOAL_POSITION_SIZE,
            dxl_id  = self.ax12a_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_position[dxl_id]) for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write position failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_GOAL_POSITION_ADDR,
            size    = DXL_MX28_GOAL_POSITION_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_position[dxl_id]) for dxl_id in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write position failed')



    def writeGoalSpeed(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_GOAL_SPEED_ADDR,
            size    = DXL_XL320_GOAL_SPEED_SIZE,
            dxl_id  = self.xl320_id_list,
            params  = [self.dxl_controller2.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write speed failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_GOAL_SPEED_ADDR,
            size    = DXL_AX12A_GOAL_SPEED_SIZE,
            dxl_id  = self.ax12a_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write speed failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_GOAL_SPEED_ADDR,
            size    = DXL_MX28_GOAL_SPEED_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write speed failed')


    
    def readPresentPosition(self) -> None:
        ERR_HANDLE_VAL = 999999

        for dxl_id in self.xl320_id_list:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_POSITION_ADDR,
                size    = DXL_XL320_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_position[dxl_id] = res

        for dxl_id in self.ax12a_id_list:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_POSITION_ADDR,
                size    = DXL_AX12A_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_position[dxl_id] = res

        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_POSITION_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_POSITION_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            if res[i] >= DXL_OK:
                self.present_position[self.mx28_id_list[i]] = res[i]



    def goalPositionSubCallback(self, msg:adisha_interfaces.JointPosition) -> None:
        for i in range(len(msg.dxl_id)):
            self.goal_position[msg.dxl_id[i]] = msg.val[i]

        with self.dxl_write_mutex:
            self.writeGoalPosition()



    def goalSpeedSubCallback(self, msg:adisha_interfaces.JointVelocity) -> None:
        for i in range(len(msg.dxl_id)):
            self.goal_speed[msg.dxl_id[i]] = msg.val[i]

        with self.dxl_write_mutex:
            self.writeGoalSpeed()