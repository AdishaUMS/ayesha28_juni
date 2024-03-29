from setuptools import setup

package_name = 'adisha_tests'
package_list = [
    package_name,
    package_name + '/modules',
    package_name + '/modules/dxl_tests'
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
            'rclnode_test = adisha_tests.rclnode_test:main',
            'custom_msg_test = adisha_tests.custom_msg_test:main',
            'dxl_xl320_test = adisha_tests.dxl_xl320_test:main',
            'dxl_ax12a_test = adisha_tests.dxl_ax12a_test:main',
            'dxl_mx28_test = adisha_tests.dxl_mx28_test:main'
        ],
    },
)
