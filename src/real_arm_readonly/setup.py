from setuptools import setup

setup(
    name='real_arm_readonly', version='0.1.0', packages=['real_arm_readonly'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/real_arm_readonly']),
        ('share/real_arm_readonly', ['package.xml']),
        ('share/real_arm_readonly/config', [
            '../arm-platform/config/arm6.urdf',
            '../arm-platform/config/real_arm_readonly.rviz']),
        ('share/real_arm_readonly/launch', [
            '../arm-platform/launch/real_arm_readonly_rviz.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Windy Arm', maintainer_email='maintainer@example.invalid',
    description='Isolated receive-only gamma6 encoder visualization', license='Apache-2.0',
    entry_points={'console_scripts': ['feedback = real_arm_readonly.feedback_node:main']},
)
