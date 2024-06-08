import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/robot_config.yaml')
CALIB_DATA_PATH     = os.path.join(os.getcwd(), 'src/adisha_data/data/calibration/mpu6050.yaml')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']
    MASTER_CLOCK    = ROBOT_CONFIG['master_clock']


def generate_launch_description():

    mpu6050_controller_node = Node(
        package     = 'adisha_controllers',
        executable  = 'mpu6050_calibrator',
        name        = f'{ROBOT_ID}_mpu6050_calibrator',
        parameters  = [
            {'id': ROBOT_ID},
            {'imu_period': MASTER_CLOCK},
            {'sample_num': 100},
            {'calib_data_path': CALIB_DATA_PATH}
        ]
    )

    return LaunchDescription([
        mpu6050_controller_node
    ])