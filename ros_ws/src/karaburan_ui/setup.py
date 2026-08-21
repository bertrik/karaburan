from glob import glob

from setuptools import find_packages, setup


package_name = 'karaburan_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='mleegwt@users.noreply.github.com',
    description='Operator interfaces for the Karaburan boat.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leaflet_map = karaburan_ui.leaflet_map:main',
        ],
    },
)
