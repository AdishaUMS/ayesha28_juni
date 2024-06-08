import smbus2
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
import adisha_interfaces.msg as adisha_interfaces



class MPU6050ControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('MPU6050ControllerNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('imu_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sample_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('calib_data_path', rclpy.Parameter.Type.STRING)

        self.ID                 = self.get_parameter('id').value
        self.IMU_PERIOD         = self.get_parameter('imu_period').value
        self.SAMPLE_NUM         = self.get_parameter('sample_num').value
        self.CALIB_DATA_PATH    = self.get_parameter('calib_data_path').value
        self.CALIB_DATA_YAML    = None

        try:
            with open(self.CALIB_DATA_PATH, 'r') as file:
                self.CALIB_DATA_YAML = yaml.safe_load(file)

        except:
            self.get_logger().error('Failed to get MPU6050 calibration file!')
            self.get_logger().error(f'Please make sure {self.CALIB_DATA_PATH} exists')
            self.destroy_node()
            exit()

        self.GYRO_X_OFFSET  = self.CALIB_DATA_YAML['gyro_val'][0]
        self.GYRO_Y_OFFSET  = self.CALIB_DATA_YAML['gyro_val'][1]
        self.GYRO_Z_OFFSET  = self.CALIB_DATA_YAML['gyro_val'][2]

        self.I2C_ADDR       = 0x68
        self.PWR_MGMT_1     = 0x6B
        self.SMPLRT_DIV     = 0x19
        self.CONFIG         = 0x1A
        self.GYRO_CONFIG    = 0x1B
        self.INT_ENABLE     = 0x38
        self.ACCEL_XOUT_H   = 0x3B
        self.ACCEL_YOUT_H   = 0x3D
        self.ACCEL_ZOUT_H   = 0x3F
        self.GYRO_XOUT_H    = 0x43
        self.GYRO_YOUT_H    = 0x45
        self.GYRO_ZOUT_H    = 0x47

        self.ACCEL_COMP_FILT_CONST      = 0.7
        self.GYRO_COMP_FILT_CONST       = 0.7
        self.ACCEL_GYRO_COMP_FILT_CONST = 0.96

        self.accel_val  = np.array([[0.0], [0.0], [0.0]])
        self.gyro_val   = np.array([[0.0], [0.0], [0.0]])
        self.roll       = 0.0
        self.pitch      = 0.0
        self.yaw        = 0.0

        self.smbus = smbus2.SMBus(1)
        self.smbus.write_byte_data(self.I2C_ADDR, self.SMPLRT_DIV, 7)
        self.smbus.write_byte_data(self.I2C_ADDR, self.PWR_MGMT_1, 1)
        self.smbus.write_byte_data(self.I2C_ADDR, self.CONFIG, 0)
        self.smbus.write_byte_data(self.I2C_ADDR, self.GYRO_CONFIG, 24)
        self.smbus.write_byte_data(self.I2C_ADDR, self.INT_ENABLE, 1)

        self.inertial_pub = self.create_publisher(
            msg_type    = adisha_interfaces.Inertial,
            topic       = f'{self.ID}/inertial',
            qos_profile = 1000
        )

        self.imu_controller_timer = self.create_timer(
            self.IMU_PERIOD,
            self.imuControllerTimerCallback
        )



    def readRawData(self, addr) -> int:
        high    = self.smbus.read_byte_data(self.I2C_ADDR, addr)
        low     = self.smbus.read_byte_data(self.I2C_ADDR, addr + 1)
        value   = (high << 8) | low

        if value > 32768:
            value -= 65536
        
        return value
    


    def getGyroDeg(self) -> np.ndarray:
        raw_gyro_x = self.readRawData(self.GYRO_XOUT_H) - self.GYRO_X_OFFSET
        raw_gyro_y = self.readRawData(self.GYRO_YOUT_H) - self.GYRO_Y_OFFSET
        raw_gyro_z = self.readRawData(self.GYRO_ZOUT_H) - self.GYRO_Z_OFFSET

        return np.array([
            [raw_gyro_x / 131.0],
            [raw_gyro_y / 131.0],
            [raw_gyro_z / 131.0]
        ])
    


    def getGyroRad(self) -> np.ndarray:
        return self.getGyroDeg()*0.01745329251
    


    def getAccel(self) -> np.ndarray:
        raw_accel_x = self.readRawData(self.ACCEL_XOUT_H)
        raw_accel_y = self.readRawData(self.ACCEL_YOUT_H)
        raw_accel_z = self.readRawData(self.ACCEL_ZOUT_H)

        return np.array([
            [raw_accel_x / 16384.0],
            [raw_accel_y / 16384.0],
            [raw_accel_z / 16384.0]
        ])
    


    def calibrateGyro(self) -> None:
        gyro_x_offset = 0
        gyro_y_offset = 0
        gyro_z_offset = 0

        for i in range(self.SAMPLE_NUM):
            gyro_x_offset += self.readRawData(self.GYRO_XOUT_H)
            gyro_y_offset += self.readRawData(self.GYRO_YOUT_H)
            gyro_z_offset += self.readRawData(self.GYRO_ZOUT_H)
            time.sleep(0.01)

        gyro_x_offset /= self.SAMPLE_NUM
        gyro_y_offset /= self.SAMPLE_NUM
        gyro_z_offset /= self.SAMPLE_NUM

        calibration_data = {
            'gyro_val': [
                int(gyro_x_offset),
                int(gyro_y_offset),
                int(gyro_z_offset)
            ]
        }

        with open(self.CALIB_DATA_PATH, 'w') as file:
            yaml.safe_dump(calibration_data, file)

        self.get_logger().info('MPU6050 calibrated successfully')
        self.get_logger().info(f'Calibration data saved at:\n{self.CALIB_DATA_PATH}')



    def imuControllerTimerCallback(self) -> None:
        self.accel_val  = self.getAccel()*self.ACCEL_COMP_FILT_CONST + self.accel_val*(1.0 - self.ACCEL_COMP_FILT_CONST)
        self.gyro_val   = self.getGyroRad()*self.GYRO_COMP_FILT_CONST + self.gyro_val*(1.0 - self.GYRO_COMP_FILT_CONST)

        ax      = self.accel_val[0].item()
        ay      = self.accel_val[1].item()
        az      = self.accel_val[2].item()
        gx      = self.gyro_val[0].item()
        gy      = self.gyro_val[1].item()
        gz      = self.gyro_val[2].item()
        alpha   = self.ACCEL_COMP_FILT_CONST

        ar  = np.arctan2(ay, np.sqrt(ax*ax + az*az))
        ap  = np.arctan2(-ax, np.sqrt(ay*ay + az*az))
        dgr = gy*self.IMU_PERIOD
        dgp = gx*self.IMU_PERIOD
        dgy = gz*self.IMU_PERIOD
        
        self.roll   = (1.0 - alpha)*(self.roll + dgr) + alpha*ar
        self.pitch  = (1.0 - alpha)*(self.pitch + dgp) + alpha*ap
        self.yaw    = self.yaw + dgy

        inertial_msg        = adisha_interfaces.Inertial()
        inertial_msg.roll   = self.roll
        inertial_msg.pitch  = self.pitch
        inertial_msg.yaw    = self.yaw

        self.inertial_pub.publish(inertial_msg)