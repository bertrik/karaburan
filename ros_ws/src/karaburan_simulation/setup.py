from glob import glob

from setuptools import find_packages, setup


package_name = 'karaburan_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/generated', glob('generated/*')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='mleegwt@users.noreply.github.com',
    description='Gazebo simulation environment for the Karaburan boat.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_ravensberg_scenario = '
            'karaburan_simulation.ravensberg_scenario:main',
            'simcontrol_node = karaburan_simulation.simcontrolnode:main',
        ],
    },
)
