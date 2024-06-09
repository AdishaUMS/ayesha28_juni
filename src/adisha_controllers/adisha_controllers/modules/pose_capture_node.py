import os
import yaml
import time
import RPi.GPIO as gpio
import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *
import adisha_interfaces.msg as adisha_interfaces



class PoseCaptureNode(Node):

    def __init__(self) -> None:
        super().__init__('PoseCaptureNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_baudrate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_u2d2_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('dxl_type', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joint_name', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('master_clock', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pc_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('filename', rclpy.Parameter.Type.STRING)
        self.declare_parameter('torque_enable', rclpy.Parameter.Type.INTEGER_ARRAY)

        self.ID             = self.get_parameter('id').value
        self.DXL_BAUDRATE   = self.get_parameter('dxl_baudrate').value
        self.DXL_U2D2_PORT  = self.get_parameter('dxl_u2d2_port').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
        self.JOINT_NAME     = self.get_parameter('joint_name').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        self.PC_PATH        = self.get_parameter('pc_path').value
        self.FILENAME       = self.get_parameter('filename').value
        self.TORQUE_ENABLE  = self.get_parameter('torque_enable').value

        self.STATE_INIT     = 0
        self.STATE_LOCKSAVE = 1
        self.STATE_UNLOCK   = 2
        self.state          = self.STATE_INIT
        self.state_change   = True

        self.red_btn_st     = 1
        self.black_btn_st   = 1
        self.torque_enable  = dict(())
        self.captured_pos   = dict(())

        for i in range(self.DXL_NUM):
            self.torque_enable.update({self.DXL_ID[i]: self.TORQUE_ENABLE[i]})
            self.captured_pos.update({self.DXL_ID[i]: 0})

        self.xl320_id_list  = []
        self.xl320_id_set   = set(())
        self.xl320_name     = dict(())

        self.ax12a_id_list  = []
        self.ax12a_id_set   = set(())
        self.ax12a_name     = dict(())

        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())

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

        self.button_input_sub = self.create_subscription(
            msg_type    = adisha_interfaces.Button,
            topic       = f'{self.ID}/button_input',
            callback    = self.buttonInputSubCallback,
            qos_profile = 1000
        )

        self.pose_capture_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.poseCaptureTimerCallback
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


    
    def enableTorque(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_TORQUE_ENABLE_ADDR,
            size    = DXL_XL320_TORQUE_ENABLE_SIZE,
            dxl_id  = self.xl320_id_list,
            params   = [[self.torque_enable[dxl_id]] for dxl_id in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_TORQUE_ENABLE_ADDR,
            size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
            dxl_id  = self.ax12a_id_list,
            params   = [[self.torque_enable[dxl_id]] for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            params   = [[self.torque_enable[dxl_id]] for dxl_id in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write torque failed')



    def enableAllTorque(self) -> None:
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



    def disableAllTorque(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_TORQUE_ENABLE_ADDR,
            size    = DXL_XL320_TORQUE_ENABLE_SIZE,
            dxl_id  = self.xl320_id_list,
            params   = [[0] for __ in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_TORQUE_ENABLE_ADDR,
            size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
            dxl_id  = self.ax12a_id_list,
            params   = [[0] for __ in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            params   = [[0] for __ in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write torque failed')



    def readPosition(self) -> None:
        ERR_HANDLE_VAL = 999999

        for dxl_id in self.xl320_id_list:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_POSITION_ADDR,
                size    = DXL_XL320_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res < DXL_OK:
                self.captured_pos[dxl_id] = ERR_HANDLE_VAL

            else:
                self.captured_pos[dxl_id] = res

        for dxl_id in self.ax12a_id_list:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_POSITION_ADDR,
                size    = DXL_AX12A_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res < DXL_OK:
                self.captured_pos[dxl_id] = ERR_HANDLE_VAL

            else:
                self.captured_pos[dxl_id] = res

        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_POSITION_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_POSITION_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.captured_pos[self.mx28_id_list[i]] = res[i]



    def buttonInputSubCallback(self, msg:adisha_interfaces.Button) -> None:
        self.red_btn_st     = msg.val[0]
        self.black_btn_st   = msg.val[1]



    def poseCaptureTimerCallback(self) -> None:
        if self.state == self.STATE_INIT:
            self.dxlSearch()
            self.enableTorque()
            self.state = self.STATE_LOCKSAVE


        elif self.state == self.STATE_LOCKSAVE:
            if self.state_change:
                self.state_change = False
                self.get_logger().info('\n[1] press RED to LOCK & SAVE\n[2] press BLACK to QUIT')

            if self.black_btn_st == 0:
                self.get_logger().error('Pose capture terminated!')
                quit()

            if self.red_btn_st == 0:
                self.enableAllTorque()
                self.readPosition()

                captured_pos_val = []
                for dxl_id in self.DXL_ID:
                    captured_pos_val.append(self.captured_pos[dxl_id])

                yaml_data = {'val': captured_pos_val}
                with open(os.path.join(self.PC_PATH, self.FILENAME), 'w') as file:
                    yaml.safe_dump(yaml_data, file)

                self.state          = self.STATE_UNLOCK
                self.state_change   = True
                
                time.sleep(1.0)
                self.red_btn_st     = 1
                self.black_btn_st   = 1


        elif self.state == self.STATE_UNLOCK:
            if self.state_change:
                self.state_change = False
                self.get_logger().info('\n[1] press RED to UNLOCK TORQUE\n[2] press BLACK to QUIT')
            
            if self.black_btn_st == 0:
                self.get_logger().error('Pose capture terminated!')
                quit()

            elif self.red_btn_st == 0:
                self.disableAllTorque()
                self.get_logger().error('Pose capture terminated!')
                quit()