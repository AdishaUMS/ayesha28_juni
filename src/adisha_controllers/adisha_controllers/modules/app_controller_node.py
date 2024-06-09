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
        
        self.xl320_id_list  = []
        self.xl320_id_set   = set(())
        self.xl320_name     = dict(())

        self.ax12a_id_list  = []
        self.ax12a_id_set   = set(())
        self.ax12a_name     = dict(())

        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())

        self.write_torque   = False
        self.write_position = False
        self.write_speed    = False

        self.read_torque    = False
        self.read_position  = False
        self.read_speed     = False
        self.read_sensor    = False

        self.goal_torque    = dict(())
        self.goal_position  = dict(())
        self.goal_speed     = dict(())

        self.present_torque         = dict(())
        self.present_position       = dict(())
        self.present_speed          = dict(())
        self.present_load           = dict(())
        self.present_voltage        = dict(())
        self.present_temperature    = dict(())

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

        self.goal_speed_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_speed',
            callback    = self.goalSpeedSubCallback,
            qos_profile = 1000
        )

        self.feature_en_sub = self.create_subscription(
            msg_type    = adisha_interfaces.FeatureEnable,
            topic       = f'{self.ID}/feature_enable',
            callback    = self.featureEnSubCallback,
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

        self.present_speed_pub = self.create_publisher(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/present_speed',
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
        for i in range(self.DXL_NUM):
            dxl_id      = self.DXL_ID[i]
            dxl_type    = self.DXL_TYPE[i]
            joint_name  = self.JOINT_NAME[i]

            self.goal_torque.update({dxl_id: 0})
            self.goal_position.update({dxl_id: 0})
            self.goal_speed.update({dxl_id: 0})
            self.present_torque.update({dxl_id: 0})
            self.present_position.update({dxl_id: 0})
            self.present_speed.update({dxl_id: 0})
            self.present_load.update({dxl_id: 0})
            self.present_voltage.update({dxl_id: 0})
            self.present_temperature.update({dxl_id: 0})

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
                    self.get_logger().info('Make sure all the servos are connected properly. Terminating.')
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
                    self.get_logger().info('Make sure all the servos are connected properly. Terminating.')
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
                    self.get_logger().info('Make sure all the servos are connected properly. Terminating.')
                    quit()
            


    def readTorque(self) -> None:
        ERR_HANDLE_VAL = 2

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_TORQUE_ENABLE_ADDR,
                size    = DXL_XL320_TORQUE_ENABLE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_torque[dxl_id] = res

            else:
                self.present_torque[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_TORQUE_ENABLE_ADDR,
                size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_torque[dxl_id] = res

            else:
                self.present_torque[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_TORQUE_ENABLE_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_TORQUE_ENABLE_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_torque[self.mx28_id_list[i]] = res[i]



    def readPresentPosition(self) -> None:
        ERR_HANDLE_VAL = 999999

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_POSITION_ADDR,
                size    = DXL_XL320_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_position[dxl_id] = res

            else:
                self.present_position[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_POSITION_ADDR,
                size    = DXL_AX12A_PRESENT_POSITION_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_position[dxl_id] = res

            else:
                self.present_position[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_POSITION_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_POSITION_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_position[self.mx28_id_list[i]] = res[i]



    def readPresentSpeed(self) -> None:
        ERR_HANDLE_VAL = 999999

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_SPEED_ADDR,
                size    = DXL_XL320_PRESENT_SPEED_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_speed[dxl_id] = res

            else:
                self.present_speed[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_SPEED_ADDR,
                size    = DXL_AX12A_PRESENT_SPEED_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_speed[dxl_id] = res

            else:
                self.present_speed[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_SPEED_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_SPEED_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_speed[self.mx28_id_list[i]] = res[i]



    def readPresentLoad(self) -> None:
        ERR_HANDLE_VAL  = -99.99
        XL320_LOAD_CONV = 0.0004
        AX12A_LOAD_CONV = 0.0015
        MX28_LOAD_CONV  = 0.0024

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_LOAD_ADDR,
                size    = DXL_XL320_PRESENT_LOAD_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_load[dxl_id] = res*XL320_LOAD_CONV if res < 1024 else (1024 - res)*XL320_LOAD_CONV

            else:
                self.present_load[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_LOAD_ADDR,
                size    = DXL_AX12A_PRESENT_LOAD_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_load[dxl_id] = res*AX12A_LOAD_CONV if res < 1024 else (1024 - res)*AX12A_LOAD_CONV

            else:
                self.present_load[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_LOAD_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_LOAD_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_load[self.mx28_id_list[i]] = res[i]*MX28_LOAD_CONV if res[i] < 1024 else (1024 - res[i])*MX28_LOAD_CONV



    def readPresentVoltage(self) -> None:
        ERR_HANDLE_VAL  = -99.99
        XL320_VOLT_CONV = 0.1
        AX12A_VOLT_CONV = 0.1
        MX28_VOLT_CONV  = 0.1

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_VOLTAGE_ADDR,
                size    = DXL_XL320_PRESENT_VOLTAGE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_voltage[dxl_id] = res*XL320_VOLT_CONV

            else:
                self.present_voltage[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_VOLTAGE_ADDR,
                size    = DXL_AX12A_PRESENT_VOLTAGE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_voltage[dxl_id] = res*AX12A_VOLT_CONV

            else:
                self.present_voltage[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_VOLTAGE_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_VOLTAGE_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_voltage[self.mx28_id_list[i]] = res[i]*MX28_VOLT_CONV



    def readPresentTemperature(self) -> None:
        ERR_HANDLE_VAL  = -99.99

        for dxl_id in self.xl320_id_set:
            res = self.dxl_controller2.read(
                address = DXL_XL320_PRESENT_TEMPERATURE_ADDR,
                size    = DXL_XL320_PRESENT_TEMPERATURE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_temperature[dxl_id] = float(res)

            else:
                self.present_temperature[dxl_id] = ERR_HANDLE_VAL


        for dxl_id in self.ax12a_id_set:
            res = self.dxl_controller1.read(
                address = DXL_AX12A_PRESENT_TEMPERATURE_ADDR,
                size    = DXL_AX12A_PRESENT_TEMPERATURE_SIZE,
                dxl_id  = dxl_id
            )

            if res >= DXL_OK:
                self.present_temperature[dxl_id] = float(res)

            else:
                self.present_temperature[dxl_id] = ERR_HANDLE_VAL


        res = self.dxl_controller1.groupBulkRead(
            address         = [DXL_MX28_PRESENT_TEMPERATURE_ADDR for __ in self.mx28_id_list],
            size            = [DXL_MX28_PRESENT_TEMPERATURE_SIZE for __ in self.mx28_id_list],
            dxl_id          = self.mx28_id_list,
            error_bypass    = True,
            error_handle_val= ERR_HANDLE_VAL
        )

        for i in range(len(self.mx28_id_list)):
            self.present_temperature[self.mx28_id_list[i]] = float(res[i])



    def writeTorque(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_TORQUE_ENABLE_ADDR,
            size    = DXL_XL320_TORQUE_ENABLE_SIZE,
            dxl_id  = self.xl320_id_list,
            params  = [[self.goal_torque[dxl_id]] for dxl_id in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_TORQUE_ENABLE_ADDR,
            size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
            dxl_id  = self.ax12a_id_list,
            params  = [[self.goal_torque[dxl_id]] for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write torque failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = [[self.goal_torque[dxl_id]] for dxl_id in self.mx28_id_list]
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
            self.get_logger().error('XL320 group sync write goal pos. failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_GOAL_POSITION_ADDR,
            size    = DXL_AX12A_GOAL_POSITION_SIZE,
            dxl_id  = self.ax12a_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_position[dxl_id]) for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write goal pos. failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_GOAL_POSITION_ADDR,
            size    = DXL_MX28_GOAL_POSITION_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_position[dxl_id]) for dxl_id in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write goal pos. failed')



    def writeGoalSpeed(self) -> None:
        res = self.dxl_controller2.groupSyncWrite(
            address = DXL_XL320_GOAL_SPEED_ADDR,
            size    = DXL_XL320_GOAL_SPEED_SIZE,
            dxl_id  = self.xl320_id_list,
            params  = [self.dxl_controller2.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.xl320_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('XL320 group sync write goal spd. failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_AX12A_GOAL_SPEED_ADDR,
            size    = DXL_AX12A_GOAL_SPEED_SIZE,
            dxl_id  = self.ax12a_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.ax12a_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('AX12A group sync write goal spd. failed')


        res = self.dxl_controller1.groupSyncWrite(
            address = DXL_MX28_GOAL_SPEED_ADDR,
            size    = DXL_MX28_GOAL_SPEED_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = [self.dxl_controller1.convert2ByteToDxl(self.goal_speed[dxl_id]) for dxl_id in self.mx28_id_list]
        )

        if res < DXL_OK:
            self.get_logger().error('MX28 group sync write goal spd. failed')



    def goalTorqueSubCallback(self, msg:adisha_interfaces.JointTorque) -> None:
        self.write_torque   = True
        key                 = list(self.goal_torque.keys())
        for i in range(self.DXL_NUM):
            self.goal_torque[key[i]] = msg.val[i]



    def goalPositionSubCallback(self, msg:adisha_interfaces.JointPosition) -> None:
        self.write_position = True
        key                 = list(self.goal_position.keys())
        for i in range(self.DXL_NUM):
            self.goal_position[key[i]] = msg.val[i]



    def goalSpeedSubCallback(self, msg:adisha_interfaces.JointVelocity) -> None:
        self.write_speed    = True
        key                 = list(self.goal_speed.keys())
        for i in range(self.DXL_NUM):
            self.goal_speed[key[i]] = msg.val[i]



    def featureEnSubCallback(self, msg:adisha_interfaces.FeatureEnable) -> None:
        self.read_torque    = msg.torque_r
        self.read_position  = msg.position_r
        self.read_speed     = msg.speed_r
        self.read_sensor    = msg.sensor_r



    def controllerTimerCallback(self) -> None:
        if self.read_torque:
            self.readTorque()

            present_torque_msg          = adisha_interfaces.JointTorque()
            present_torque_msg.dxl_id   = list(self.present_torque.keys())
            present_torque_msg.val      = list(self.present_torque.values())

            self.present_torque_pub.publish(present_torque_msg)


        if self.read_position:
            self.readPresentPosition()

            present_position_msg        = adisha_interfaces.JointPosition()
            present_position_msg.dxl_id = list(self.present_position.keys())
            present_position_msg.val    = list(self.present_position.values())

            self.present_position_pub.publish(present_position_msg)


        if self.read_speed:
            self.readPresentSpeed()

            present_speed_msg           = adisha_interfaces.JointVelocity()
            present_speed_msg.dxl_id    = list(self.present_speed.keys())
            present_speed_msg.val       = list(self.present_speed.values())

            self.present_speed_pub.publish(present_speed_msg)


        if self.read_sensor:
            self.readPresentLoad()
            self.readPresentVoltage()
            self.readPresentTemperature()

            joint_sensor_msg                = adisha_interfaces.JointSensor()
            joint_sensor_msg.dxl_id         = list(self.present_load.keys())
            joint_sensor_msg.load           = list(self.present_load.values())
            joint_sensor_msg.voltage        = list(self.present_voltage.values())
            joint_sensor_msg.temperature    = list(self.present_temperature.values())

            self.joint_sensor_pub.publish(joint_sensor_msg)


        if self.write_torque:
            self.write_torque = False
            self.writeTorque()


        if self.write_position:
            self.write_position = False
            self.writeGoalPosition()


        if self.write_speed:
            self.write_speed = False
            self.writeGoalSpeed()