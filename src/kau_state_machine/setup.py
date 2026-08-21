from glob import glob

from setuptools import find_packages, setup

package_name = 'kau_state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeon Semin',
    maintainer_email='eiffeltower1206@gmail.com',
    description='KAU AMET 상태 머신 (system_supervisor + state_machine)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_supervisor = kau_state_machine.nodes.system_supervisor:main',
            'state_machine = kau_state_machine.nodes.state_machine:main',
        ],
    },
)
