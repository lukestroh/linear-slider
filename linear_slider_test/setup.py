from setuptools import find_packages, setup

package_name = 'linear_slider_test'

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
    maintainer='lukestroh',
    maintainer_email='luke.strohbehn@gmail.com',
    description='Test package for the linear slider',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_velocity_controller_node = linear_slider_test.test_velocity_controller_node:main',
            'test_joint_trajectory_controller_node = linear_slider_test.test_joint_trajectory_controller_node:main'
        ],
    },
)
