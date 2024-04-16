import os
import socket
import yaml



OS_PATH                 = '/adisha-os'
ADISHA_CONTROLLERS_PATH = os.path.join(OS_PATH, 'src/adisha_controllers')
ADISHA_INTERFACES_PATH  = os.path.join(OS_PATH, 'src/adisha_interfaces')
ADISHA_DATA_PATH        = os.path.join(OS_PATH, 'src/adisha_data')
ADISHA_MAIN_PATH        = os.path.join(OS_PATH, 'src/adisha_main')
ADISHA_TESTS_PATH       = os.path.join(OS_PATH, 'src/adisha_tests')

HOST_OS_PATH            = os.getcwd()
HOST_CORE_CONFIG_PATH   = os.path.join(HOST_OS_PATH, 'src/adisha_data/config/core_config.yaml')
HOST_ROBOT_CONFIG_PATH  = os.path.join(HOST_OS_PATH, 'src/adisha_data/config/robot_config.yaml')



def main(args=None) -> None:
    print('----------------------------------')
    print('         ADISHA-OS SETUP          ')
    print('----------------------------------')


    # Set the ID or namespace
    print('[1/4] Insert the ID or namespace for this machine: ')
    id = input('>> ')

    print('[2/4] Insert the master clock frequency for this machine: ')
    fc = float(input('>> '))


    # Get the local IP
    print('[3/4] Insert the local network')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0)
    
    try:
        sock.connect(('10.254.254.254', 1))
        ip_addr = sock.getsockname()[0]
    
    except Exception:
        ip_addr = '127.0.0.1'
    
    finally:
        sock.close()
    
    ip = ip_addr
    print(f'>> {ip}')

    # Config file setup
    print('[4/4] Setting up config files...')
    
    core_config_yaml = {
        'os_path': OS_PATH,
        'adisha_controllers_path': ADISHA_CONTROLLERS_PATH,
        'adisha_interfaces_path': ADISHA_INTERFACES_PATH,
        'adisha_data_path': ADISHA_DATA_PATH,
        'adisha_main_path': ADISHA_MAIN_PATH,
        'adisha_tests_path': ADISHA_TESTS_PATH
    }
    with open(HOST_CORE_CONFIG_PATH, 'w') as file:
        yaml.dump(core_config_yaml, file)

    robot_config_yaml = {
        'id': id,
        'ip': ip,
        'master_clock': 1./fc
    }
    with open(HOST_ROBOT_CONFIG_PATH, 'w') as file:
        yaml.dump(robot_config_yaml, file)

    print('>> DONE')



if __name__ == '__main__':
    main()