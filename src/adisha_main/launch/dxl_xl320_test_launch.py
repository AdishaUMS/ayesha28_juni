import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


with open(os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml'), 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']


with open(os.path.join(os.getcwd(), 'src/altair_data/config/joint_config.yaml'), 'r') as file:
    JOINT_CONFIG    = yaml.safe_load(file)
    JOINT_NUM       = JOINT_CONFIG['joint_num']
    JOINT_ID        = JOINT_CONFIG['joint_id']
    JOINT_SERVO     = JOINT_CONFIG['joint_servo']


def generate_launch_description():

    dxl_xl320_test_node = Node(
        package     = 'adisha_tests',
        executable  = 'dxl_xl320_test',
        name        = f'{ROBOT_ID}_dxl_xl320_test',
        parameters  = [
            {'pub_topic': f'{ROBOT_ID}/present_position'},
            {'joint_num': JOINT_NUM},
            {'joint_id': JOINT_ID},
            {'joint_servo': JOINT_SERVO}
        ]
    )

    return LaunchDescription([
        dxl_xl320_test_node
    ])