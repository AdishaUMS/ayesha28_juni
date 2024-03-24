import rclpy
from rclpy.node import Node
import dynamixel_sdk as dxl
import adisha_interfaces.msg as adisha_interfaces



DXL_AX12A_MODEL_ADDR        = 0
DXL_AX12A_MODEL_SIZE        = 2
DXL_AX12A_TORQUE_ADDR       = 24
DXL_AX12A_TORQUE_SIZE       = 1
DXL_AX12A_GOAL_POS_ADDR     = 30
DXL_AX12A_GOAL_POS_SIZE     = 2
DXL_AX12A_PRES_POS_ADDR     = 36
DXL_AX12A_PRES_POS_SIZE     = 2



class DxlAx12aSyncRWTestNode(Node):

    def __init__(self) -> None:
        super().__init__('DxlAx12aSyncRWTestNode')
        self.get_logger().info('-----------[ DXL AX12A Sync RW Test ]-----------')

        self.declare_parameter('pub_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('joint_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('joint_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('joint_servo', rclpy.Parameter.Type.STRING_ARRAY)

        self.pub_topic_param    = self.get_parameter('pub_topic').value
        self.joint_num_param    = self.get_parameter('joint_num').value
        self.joint_id_param     = self.get_parameter('joint_id').value
        self.joint_servo_param  = self.get_parameter('joint_servo').value

        self.joint_id       = []

        self.goal_pos_amp   = 300
        self.timestep       = 0.2
        self.movestep       = 4
        self.slope          = True
        self.current_idx    = 0
        self.current_amp    = 0

        self.action_timer   = self.create_timer(self.timestep, self.actionCallback)
        self.pos_pub        = self.create_publisher(adisha_interfaces.JointPosition, self.pub_topic_param, 1000)

        self.porthandler    = dxl.PortHandler('/dev/ttyUSB0')
        self.packethandler  = dxl.PacketHandler(1.0)

        self.sync_w_torque = dxl.GroupSyncWrite(
            self.porthandler,
            self.packethandler,
            DXL_AX12A_TORQUE_ADDR,
            DXL_AX12A_TORQUE_SIZE
        )

        self.sync_w_pos = dxl.GroupSyncWrite(
            self.porthandler,
            self.packethandler,
            DXL_AX12A_GOAL_POS_ADDR,
            DXL_AX12A_GOAL_POS_SIZE
        )

        self.sync_r_pos = dxl.GroupBulkRead(
            self.porthandler,
            self.packethandler
        )

        try:
            self.porthandler.openPort()
            self.get_logger().info('Port opened successfully')
        except:
            self.get_logger().error('Failed to open port, try chmod-ing the port')
            self.get_logger().warn('Test terminated!')
            quit()

        try:
            self.porthandler.setBaudRate(1000000)
            self.get_logger().info('Baudrate set successfully')
        except:
            self.get_logger().error('Failed to set the baudrate')
            self.get_logger().warn('Test terminated!')
            quit()


    
    def dxlSearch(self) -> None:
        for i in range(self.joint_num_param):
            if self.joint_servo_param[i] == 'AX12A':
                self.joint_id.append(self.joint_id_param[i])

        for id in self.joint_id:
            model_number, dxl_res, dxl_err = self.packethandler.read2ByteTxRx(self.porthandler, id, DXL_AX12A_MODEL_ADDR)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()

            else:
                self.get_logger().info(f'ID#{id}: Connected')
                dxl_res = self.sync_r_pos.addParam(id, DXL_AX12A_PRES_POS_ADDR, DXL_AX12A_PRES_POS_SIZE)

                if dxl_res != True:
                    self.get_logger().error(f'ID#{id}: Adding pres. pos. param failed')
                    quit()



    def enableTorque(self) -> None:
        for id in self.joint_id:
            dxl_res = self.sync_w_torque.addParam(id, [1])
            
            if dxl_res != True:
                self.get_logger().error(f'ID#{id}: Adding torque param failed')
                quit()

        dxl_res = self.sync_w_torque.txPacket()

        if dxl_res != dxl.COMM_SUCCESS:
            self.get_logger().error(f'Torque enable failed: {self.packethandler.getTxRxResult(dxl_res)}')
            quit()

        else:
            self.get_logger().info('All torques enabled')

        self.sync_w_torque.clearParam()



    def disableTorque(self) -> None:
        for id in self.joint_id:
            dxl_res = self.sync_w_torque.addParam(id, [0])
            
            if dxl_res != True:
                self.get_logger().error(f'ID#{id}: Adding torque param failed')
                quit()

        dxl_res = self.sync_w_torque.txPacket()

        if dxl_res != dxl.COMM_SUCCESS:
            self.get_logger().error(f'Torque disable failed: {self.packethandler.getTxRxResult(dxl_res)}')
            quit()

        else:
            self.get_logger().info('All torques disabled')

        self.sync_w_torque.clearParam()



    def readPos(self) -> None:
        dxl_res = self.sync_r_pos.txRxPacket()
        
        if dxl_res != dxl.COMM_SUCCESS:
            self.get_logger().error(f'{self.packethandler.getTxRxResult(dxl_res)}')

        pres_pos = []

        for id in self.joint_id:
            dxl_res = self.sync_r_pos.isAvailable(id, DXL_AX12A_PRES_POS_ADDR, DXL_AX12A_PRES_POS_SIZE)

            if dxl_res != True:
                self.get_logger().error(f'ID#{id}: Failed to read pos')
                quit()

            pres_pos.append(
                self.sync_r_pos.getData(id, DXL_AX12A_PRES_POS_ADDR, DXL_AX12A_PRES_POS_SIZE)
            )

        pub_msg = adisha_interfaces.JointPosition()
        pub_msg.val = pres_pos
        self.pos_pub.publish(pub_msg)



    def writePos(self) -> None:
        if self.slope:
            self.current_amp += self.movestep

            if self.current_amp >= self.goal_pos_amp:
                self.slope = False

        else :
            self.current_amp -= self.movestep

            if self.current_amp <= 0:
                self.slope = True
                self.current_idx = (self.current_idx + 1)%len(self.joint_id)


        for id in self.joint_id:

            if id == self.joint_id[self.current_idx]:
                dxl_res = self.sync_w_pos.addParam(
                    id, [
                        dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.current_amp)),
                        dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.current_amp))
                    ]
                )

                if dxl_res != True:
                    self.get_logger().error(f'ID#{id}: Adding goal pos. param failed')
                    quit()

            else:
                dxl_res = self.sync_w_pos.addParam(
                    id, [
                        dxl.DXL_LOBYTE(dxl.DXL_LOWORD(0)),
                        dxl.DXL_HIBYTE(dxl.DXL_LOWORD(0))
                    ]
                )

                if dxl_res != True:
                    self.get_logger().error(f'ID#{id}: Adding goal pos. param failed')
                    quit()

        dxl_res = self.sync_w_pos.txPacket()

        if dxl_res != dxl.COMM_SUCCESS:
            self.get_logger().error(f'{self.packethandler.getTxRxResult(dxl_res)}')

        self.sync_w_pos.clearParam()



    def start(self) -> None:
        self.dxlSearch()
        self.enableTorque()



    def actionCallback(self) -> None:
        self.writePos()
        self.readPos()



    def stop(self) -> None:
        self.disableTorque()




class DxlAx12aRWTestNode(Node):

    def __init__(self) -> None:
        super().__init__('DxlAx12aRWTestNode')
        self.get_logger().info('-----------[ DXL AX12A RW Test ]-----------')

        self.declare_parameter('pub_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('joint_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('joint_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('joint_servo', rclpy.Parameter.Type.STRING_ARRAY)

        self.pub_topic_param    = self.get_parameter('pub_topic').value
        self.joint_num_param    = self.get_parameter('joint_num').value
        self.joint_id_param     = self.get_parameter('joint_id').value
        self.joint_servo_param  = self.get_parameter('joint_servo').value

        self.joint_id       = []

        self.goal_pos_amp   = 300
        self.timestep       = 0.2
        self.movestep       = 4
        self.slope          = True
        self.current_idx    = 0
        self.current_amp    = 0

        self.action_timer   = self.create_timer(self.timestep, self.actionCallback)
        self.pos_pub        = self.create_publisher(adisha_interfaces.JointPosition, self.pub_topic_param, 1000)

        self.porthandler    = dxl.PortHandler('/dev/ttyUSB0')
        self.packethandler  = dxl.PacketHandler(1.0)

        try:
            self.porthandler.openPort()
            self.get_logger().info('Port opened successfully')
        except:
            self.get_logger().error('Failed to open port, try chmod-ing the port')
            self.get_logger().warn('Test terminated!')
            quit()

        try:
            self.porthandler.setBaudRate(1000000)
            self.get_logger().info('Baudrate set successfully')
        except:
            self.get_logger().error('Failed to set the baudrate')
            self.get_logger().warn('Test terminated!')
            quit()



    def dxlSearch(self) -> None:
        for i in range(self.joint_num_param):
            if self.joint_servo_param[i] == 'AX12A':
                self.joint_id.append(self.joint_id_param[i])

        for id in self.joint_id:
            model_number, dxl_res, dxl_err = self.packethandler.read2ByteTxRx(self.porthandler, id, DXL_AX12A_MODEL_ADDR)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()

            else:
                self.get_logger().info(f'ID#{id}: Connected')



    def enableTorque(self) -> None:
        for id in self.joint_id:
            dxl_res, dxl_err = self.packethandler.write1ByteTxRx(self.porthandler, id, DXL_AX12A_TORQUE_ADDR, 1)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()

            else:
                self.get_logger().info(f'ID#{id}: Torque enabled')



    def disableTorque(self) -> None:
        for id in self.joint_id:
            dxl_res, dxl_err = self.packethandler.write1ByteTxRx(self.porthandler, id, DXL_AX12A_TORQUE_ADDR, 0)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()

            else:
                self.get_logger().info(f'ID#{id}: Torque disabled')



    def readPos(self) -> None:
        pres_pos_list = []

        for id in self.joint_id:
            pres_pos, dxl_res, dxl_err = self.packethandler.read2ByteTxRx(self.porthandler, id, DXL_AX12A_PRES_POS_ADDR)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()

            else:
                pres_pos_list.append(pres_pos)

        pub_msg = adisha_interfaces.JointPosition()
        pub_msg.val = pres_pos_list
        self.pos_pub.publish(pub_msg)



    def writePos(self) -> None:
        if self.slope:
            self.current_amp += self.movestep

            if self.current_amp >= self.goal_pos_amp:
                self.slope = False

        else :
            self.current_amp -= self.movestep

            if self.current_amp <= 0:
                self.slope = True
                self.current_idx = (self.current_idx + 1)%len(self.joint_id)

        for id in self.joint_id:

            if id == self.joint_id[self.current_idx]:
                dxl_res, dxl_err = self.packethandler.write2ByteTxRx(self.porthandler, id, DXL_AX12A_GOAL_POS_ADDR, self.current_amp)

            else:
                dxl_res, dxl_err = self.packethandler.write2ByteTxRx(self.porthandler, id, DXL_AX12A_GOAL_POS_ADDR, 0)

            if dxl_res != dxl.COMM_SUCCESS:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getTxRxResult(dxl_res)}')
                quit()

            elif dxl_err != 0:
                self.get_logger().error(f'ID#{id}: {self.packethandler.getRxPacketError(dxl_err)}')
                quit()



    def start(self) -> None:
        self.dxlSearch()
        self.enableTorque()



    def actionCallback(self) -> None:
        self.writePos()
        self.readPos()



    def stop(self) -> None:
        self.disableTorque()