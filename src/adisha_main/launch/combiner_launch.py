import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


JOINT_CONFIG_PATH       = os.path.join(os.getcwd(), 'src/adisha_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH       = os.path.join(os.getcwd(), 'src/adisha_data/config/robot_config.yaml')
COMBINER_CONFIG_PATH    = os.path.join(os.getcwd(), 'src/adisha_data/data/motion/combiner_config.yaml')
MOTSEQ_PATH             = os.path.join(os.getcwd(), 'src/adisha_data/data/app_motion_sequencer')
POSE_STUDIO_PATH        = os.path.join(os.getcwd(), 'src/adisha_data/data/app_pose_studio')
OUTPUT_PATH             = os.path.join(os.getcwd(), 'src/adisha_data/data/motion')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']


with open(JOINT_CONFIG_PATH, 'r') as file:
    JOINT_CONFIG    = yaml.safe_load(file)
    DXL_ID          = JOINT_CONFIG['dxl_id']
    DXL_NUM         = JOINT_CONFIG['dxl_num']


with open(COMBINER_CONFIG_PATH, 'r') as file:
    COMBINER_CONFIG = yaml.safe_load(file)
    OUTPUT_PATH     = os.path.join(OUTPUT_PATH, COMBINER_CONFIG['output_filename'])
    ARM_FILENAME    = os.path.join(MOTSEQ_PATH, COMBINER_CONFIG['arm'])
    LEG_FILENAME    = os.path.join(MOTSEQ_PATH, COMBINER_CONFIG['leg'])
    DT_MS           = COMBINER_CONFIG['dt_ms']
    Q_PROP          = COMBINER_CONFIG['q_prop']


def generate_launch_description():
    
    combiner_node = Node(
        package     = 'adisha_utilities',
        executable  = 'combiner',
        name        = f'{ROBOT_ID}_combiner',
        parameters  = [
            {'id': ROBOT_ID},
            {'dxl_id': DXL_ID},
            {'dxl_num': DXL_NUM},
            {'dt_ms': DT_MS},
            {'q_prop': Q_PROP},
            {'ps_path': POSE_STUDIO_PATH},
            {'arm_path': ARM_FILENAME},
            {'leg_path': LEG_FILENAME},
            {'output_path': OUTPUT_PATH},
            {'arm_id': [1, 12]},
            {'leg_id': [13, 24]}
        ]
    )

    return LaunchDescription([
        combiner_node
    ])