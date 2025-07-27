from setuptools import find_packages, setup

package_name = 'joint_pdo_sender'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yannik',
    maintainer_email='yannik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_to_pdo = joint_pdo_sender.joint_state_to_pdo:main',
            'joint_state_to_pdo_save_state = joint_pdo_sender.joint_state_to_pdo_save_state:main',
            'dummy_joint_state_publisher = joint_pdo_sender.dummy_joint_state_publisher:main',
            'sdo_joint_writer = joint_pdo_sender.sdo_joint_writer:main',
        ],
    },
)
