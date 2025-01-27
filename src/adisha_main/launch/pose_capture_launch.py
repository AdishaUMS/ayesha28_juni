import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


JOINT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/robot_config.yaml')
POSE_CAPTURE_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/data/pose_capture')
PC_PARAMS_PATH      = os.path.join(os.getcwd(), 'src/adisha_data/launch_params/pose_capture.yaml')


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


with open(PC_PARAMS_PATH, 'r') as file:
    PC_CONFIG       = yaml.safe_load(file)
    FILENAME        = PC_CONFIG['filename']
    TORQUE_ENABLE   = PC_CONFIG['torque_enable']


def generate_launch_description():
    
    pose_capture_node = Node(
        package     = 'adisha_controllers',
        executable  = 'pose_capture',
        name        = f'{ROBOT_ID}_pose_capture',
        parameters  = [
            {'id': ROBOT_ID},
            {'dxl_baudrate': DXL_BAUDRATE},
            {'dxl_u2d2_port': DXL_U2D2_PORT},
            {'dxl_num': DXL_NUM},
            {'dxl_id': DXL_ID},
            {'dxl_type': DXL_TYPE},
            {'joint_name': JOINT_NAME},
            {'master_clock': MASTER_CLOCK},
            {'pc_path': POSE_CAPTURE_PATH},
            {'filename': FILENAME},
            {'torque_enable': TORQUE_ENABLE}
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

    return LaunchDescription([
        pose_capture_node,
        button_controller_node
    ])