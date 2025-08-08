from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/nav2_stack.launch.py']),
        ('share/' + package_name + '/launch', ['launch/boat.launch.py']),
        ('share/' + package_name + '/config', ['config/controller_server.yaml']),
        ('share/' + package_name + '/config', ['config/planner_server.yaml']),
        ('share/' + package_name + '/config', ['config/behavior_server.yaml']),
        ('share/' + package_name + '/config', ['config/bt_navigator.yaml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/config', ['config/navsat.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='mleegwt@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_navigation_node = navigation.boatnavigator:main',
            'fix_status_override_node = navigation.fix_status_override_node:main',
        ],
    },
)
