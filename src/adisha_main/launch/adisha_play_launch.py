import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


JOINT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/robot_config.yaml')
CTRL_CONFIG_PATH    = os.path.join(os.getcwd(), 'src/adisha_data/config/controller_config.yaml')
CALIB_DATA_PATH     = os.path.join(os.getcwd(), 'src/adisha_data/data/calibration/mpu6050.yaml')
PLAY_PARAMS_PATH    = os.path.join(os.getcwd(), 'src/adisha_data/launch_params/adisha_play.yaml')
MOTION_PATH         = os.path.join(os.getcwd(), 'src/adisha_data/data/motion')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']
    MASTER_CLOCK    = ROBOT_CONFIG['master_clock']


with open(JOINT_CONFIG_PATH, 'r') as file:
    JOINT_CONFIG    = yaml.safe_load(file)
    DXL_BAUDRATE    = JOINT_CONFIG['dxl_baudrate']
    DXL_U2D2_PORT   = JOINT_CONFIG['dxl_u2d2_port']
    DXL_NUM         = JOINT_CONFIG['dxl_num']
    DXL_ID          = JOINT_CONFIG['dxl_id']
    DXL_TYPE        = JOINT_CONFIG['dxl_type']
    JOINT_NAME      = JOINT_CONFIG['joint_name']


with open(CTRL_CONFIG_PATH, 'r') as file:
    CTRL_CONFIG = yaml.safe_load(file)
    TRACKER_KP  = CTRL_CONFIG['tracker']['kp']
    TRACKER_KI  = CTRL_CONFIG['tracker']['ki']
    TRACKER_KD  = CTRL_CONFIG['tracker']['kd']
    STABLER_KP  = CTRL_CONFIG['stabler']['kp']
    STABLER_KI  = CTRL_CONFIG['stabler']['ki']
    STABLER_KD  = CTRL_CONFIG['stabler']['kd']


with open(PLAY_PARAMS_PATH, 'r') as file:
    PLAY_PARAMS             = yaml.safe_load(file)
    SELECTED_MOTION_PATH    = os.path.join(MOTION_PATH, PLAY_PARAMS['motion'])


def generate_launch_description():

    mpu6050_controller_node = Node(
        package     = 'adisha_controllers',
        executable  = 'mpu6050_controller',
        name        = f'{ROBOT_ID}_mpu6050_controller',
        parameters  = [
            {'id': ROBOT_ID},
            {'imu_period': MASTER_CLOCK},
            {'sample_num': -1},
            {'calib_data_path': CALIB_DATA_PATH}
        ]
    )

    button_controller_node = Node(
        package     = 'adisha_controllers',
        executable  = 'button_controller',
        name        = f'{ROBOT_ID}_button_controller',
        parameters  = [
            {'id': ROBOT_ID},
            {'master_clock': MASTER_CLOCK},
            {'input_pin': [16, 26]}
        ]
    )

    motion_setter_node = Node(
        package     = 'adisha_controllers',
        executable  = 'motion_setter',
        name        = f'{ROBOT_ID}_motion_setter',
        parameters  = [
            {'id': ROBOT_ID},
            {'dxl_num': DXL_NUM},
            {'dxl_id': DXL_ID},
            {'master_clock': MASTER_CLOCK},
            {'motion_path': SELECTED_MOTION_PATH},
            {'stabler_kp': TRACKER_KP},
            {'stabler_ki': TRACKER_KI},
            {'stabler_kd': TRACKER_KD}
        ]
    )
    
    motion_player_node = Node(
        package     = 'adisha_controllers',
        executable  = 'motion_player',
        name        = f'{ROBOT_ID}_motion_player',
        parameters  = [
            {'id': ROBOT_ID},
            {'dxl_baudrate': DXL_BAUDRATE},
            {'dxl_u2d2_port': DXL_U2D2_PORT},
            {'dxl_num': DXL_NUM},
            {'dxl_id': DXL_ID},
            {'dxl_type': DXL_TYPE},
            {'joint_name': JOINT_NAME},
            {'master_clock': MASTER_CLOCK},
            {'tracker_kp': TRACKER_KP},
            {'tracker_ki': TRACKER_KI},
            {'tracker_kd': TRACKER_KD}
        ]
    )

    return LaunchDescription([
        mpu6050_controller_node,
        button_controller_node,
        motion_setter_node,
        motion_player_node
    ])