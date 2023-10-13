from setuptools import setup

package_name = 'sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkn',
    maintainer_email='ofuro.jabu.jabu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_talker = sensor.joint_talker:main',
            'sensor_listener = sensor.sensor_listener:main',
            'joint_coord_talker = sensor.joint_coord_talker:main',
            'joint_coord_listener = sensor.joint_coord_listener:main',
        ],
    },
)
