from setuptools import setup
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('urdf/*.xacro')),
        ('share/' + package_name, glob('urdf/sensors/*.xacro')),
        # ('share/' + package_name, glob('meshes/*')),
        ('share/' + package_name, glob('world/*')),
        ('share/' + package_name + "/model/small_warehouse", glob('model/small_warehouse/*')),
        ('share/' + package_name + "/model/turtlebot3_world/meshes/", glob('model/turtlebot3_world/meshes/*')),
        ('share/' + package_name + "/model/turtlebot3_world", glob('model/turtlebo/model*')),
        ('share/' + package_name, glob('urdf/*.urdf')),
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
        ],
    },
)
