from setuptools import setup, find_packages

package_name = 'system_health_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/health_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alan',
    maintainer_email='alan@todo.todo',
    description='A package to monitor system health.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_monitor = system_health_monitor.computer_monitor:main',
            'node_monitor = system_health_monitor.node_monitor:main',
            'image_monitor = system_health_monitor.image_monitor:main',
        ],
    },
)
