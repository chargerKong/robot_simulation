from setuptools import setup
from glob import glob
package_name = 'nav2_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps/' , glob('maps/*')),
        ('share/' + package_name + '/launch/' , glob('launch/*')),
        ('share/' + package_name + '/params/' , glob('params/*')),
        ('share/' + package_name + '/rviz/' , glob('rviz/*')),
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
