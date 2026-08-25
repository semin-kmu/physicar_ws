from glob import glob

from setuptools import setup

package_name = 'kau_gui'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gui.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', ['scripts/setup_venv.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeon Semin',
    maintainer_email='eiffeltower1206@kookmin.ac.kr',
    description='주행 디버깅 GUI. 관측 전용',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kau_gui = kau_gui.app:main',
        ],
    },
)
