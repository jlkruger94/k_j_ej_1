from setuptools import find_packages, setup

package_name = 'k_j_ej_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/counter_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jlkruger94',
    maintainer_email='jlkruger94@example.com',
    description='ROS 2 Jazzy counter demo package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'counter_publisher = k_j_ej_1.counter_publisher:main',
            'counter_subscriber = k_j_ej_1.counter_subscriber:main',
        ],
    },
)
