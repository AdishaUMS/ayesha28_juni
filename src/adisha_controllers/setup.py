from setuptools import setup

package_name = 'adisha_controllers'
package_list = [
    package_name,
    package_name + '/modules',
    package_name + '/modules/dxl_type'
]

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = package_list,
    data_files          = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'None',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'dxl_connection_check = adisha_controllers.modules.dxl_type.dxl_connection_check:main',
            'dxl_torque_enabler = adisha_controllers.modules.dxl_type.dxl_torque_enabler:main',
            'dxl_torque_disabler = adisha_controllers.modules.dxl_type.dxl_torque_disabler:main',
            '_dummy_app_controller = adisha_controllers._dummy_app_controller:main',
            'app_controller = adisha_controllers.app_controller:main',
            'motion_player = adisha_controllers.motion_player:main',
            'button_controller = adisha_controllers.button_controller:main',
            'mpu6050_controller = adisha_controllers.mpu6050_controller:main',
            'mpu6050_calibrator = adisha_controllers.mpu6050_calibrator:main',
            'pose_capture = adisha_controllers.pose_capture:main'
        ],
    },
)
