from setuptools import setup
from glob import glob

package_name = 'my_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz2' , glob('rviz2/*.rviz')),
        ('share/' + package_name + '/lua' , glob('lua/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kong',
    maintainer_email='chargerKong@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "myslam = my_slam.main:main",
            "icp = my_slam.main_icp:main"
        ],
    },
)
