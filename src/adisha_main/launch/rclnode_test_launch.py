import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/adisha_data/config/robot_config.yaml')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']


def generate_launch_description():

    rclnodetest1_node = Node(
        package     = 'adisha_tests',
        executable  = 'rclnode_test',
        name        = f'{ROBOT_ID}_rclnode_test1',
        parameters  = [
            {'pub_msg': 'Hello, this is TEST 1.'},
            {'pub_topic': f'{ROBOT_ID}/topic1'},
            {'sub_topic': f'{ROBOT_ID}/topic2'}
        ]
    )

    rclnodetest2_node = Node(
        package     = 'adisha_tests',
        executable  = 'rclnode_test',
        name        = f'{ROBOT_ID}_rclnode_test2',
        parameters  = [
            {'pub_msg': 'Hello, this is TEST 2.'},
            {'pub_topic': f'{ROBOT_ID}/topic2'},
            {'sub_topic': f'{ROBOT_ID}/topic1'}
        ]
    )

    return LaunchDescription([
        rclnodetest1_node,
        rclnodetest2_node
    ])