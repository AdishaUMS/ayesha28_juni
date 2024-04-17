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

        self.goal_torque    = []
        self.goal_position  = []
        self.goal_speed     = []

        self.present_torque         = []
        self.present_position       = []
        self.present_speed          = []
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

        self.goal_speed_sub = self.create_subscription(
            msg_type    = adisha_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_speed',
            callback    = self.goalSpeedSubCallback,
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
            


    def readTorque(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_TORQUE_ENABLE_ADDR,
                    size    = DXL_XL320_TORQUE_ENABLE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(2)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_TORQUE_ENABLE_ADDR,
                    size    = DXL_AX12A_TORQUE_ENABLE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(2)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_TORQUE_ENABLE_ADDR,
                    size    = DXL_MX28_TORQUE_ENABLE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(2)

        return ret_val



    def readPresentPosition(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_PRESENT_POSITION_ADDR,
                    size    = DXL_XL320_PRESENT_POSITION_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_PRESENT_POSITION_ADDR,
                    size    = DXL_AX12A_PRESENT_POSITION_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_PRESENT_POSITION_ADDR,
                    size    = DXL_MX28_PRESENT_POSITION_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

        return ret_val



    def readPresentSpeed(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_PRESENT_SPEED_ADDR,
                    size    = DXL_XL320_PRESENT_SPEED_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_PRESENT_SPEED_ADDR,
                    size    = DXL_AX12A_PRESENT_SPEED_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_PRESENT_SPEED_ADDR,
                    size    = DXL_MX28_PRESENT_SPEED_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(res)

                else:
                    ret_val.append(999999999)

        return ret_val



    def readPresentLoad(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_PRESENT_LOAD_ADDR,
                    size    = DXL_XL320_PRESENT_LOAD_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_PRESENT_LOAD_ADDR,
                    size    = DXL_AX12A_PRESENT_LOAD_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_PRESENT_LOAD_ADDR,
                    size    = DXL_MX28_PRESENT_LOAD_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

        return ret_val



    def readPresentVoltage(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_PRESENT_VOLTAGE_ADDR,
                    size    = DXL_XL320_PRESENT_VOLTAGE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_PRESENT_VOLTAGE_ADDR,
                    size    = DXL_AX12A_PRESENT_VOLTAGE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_PRESENT_VOLTAGE_ADDR,
                    size    = DXL_MX28_PRESENT_VOLTAGE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

        return ret_val



    def readPresentTemperature(self) -> list:
        ret_val = []

        for id in self.DXL_ID:
            if id in self.xl320_id_set:
                res = self.dxl_controller2.read(
                    address = DXL_XL320_PRESENT_TEMPERATURE_ADDR,
                    size    = DXL_XL320_PRESENT_TEMPERATURE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.ax12a_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_AX12A_PRESENT_TEMPERATURE_ADDR,
                    size    = DXL_AX12A_PRESENT_TEMPERATURE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

            elif id in self.mx28_id_set:
                res = self.dxl_controller1.read(
                    address = DXL_MX28_PRESENT_TEMPERATURE_ADDR,
                    size    = DXL_MX28_PRESENT_TEMPERATURE_SIZE,
                    dxl_id  = id
                )

                if res >= DXL_OK:
                    ret_val.append(float(res))

                else:
                    ret_val.append(0.0)

        return ret_val



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



    def goalTorqueSubCallback(self, msg:adisha_interfaces.JointTorque) -> None:
        self.write_torque   = True
        self.goal_torque    = msg.val



    def goalPositionSubCallback(self, msg:adisha_interfaces.JointPosition) -> None:
        self.write_position = True
        self.goal_position  = msg.val



    def goalSpeedSubCallback(self, msg:adisha_interfaces.JointVelocity) -> None:
        self.write_speed    = True
        self.goal_speed     = msg.val



    def controllerTimerCallback(self) -> None:
        present_torque_msg      = adisha_interfaces.JointTorque()
        present_position_msg    = adisha_interfaces.JointPosition()
        present_speed_msg       = adisha_interfaces.JointVelocity()
        joint_sensor_msg        = adisha_interfaces.JointSensor()

        present_torque_msg.val          = self.readTorque()
        present_position_msg.val        = self.readPresentPosition()
        present_speed_msg.val           = self.readPresentSpeed()
        joint_sensor_msg.load           = self.readPresentLoad()
        joint_sensor_msg.voltage        = self.readPresentVoltage()
        joint_sensor_msg.temperature    = self.readPresentTemperature()

        self.present_torque_pub.publish(present_torque_msg)
        self.present_position_pub.publish(present_position_msg)
        self.present_speed_pub.publish(present_speed_msg)
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