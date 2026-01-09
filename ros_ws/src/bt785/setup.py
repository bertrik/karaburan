from setuptools import find_packages, setup

package_name = 'bt785'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'bleak',
    ],
    zip_safe=True,
    maintainer='Bertrik Sikken',
    maintainer_email='bertrik@sikken.nl',
    description='BT785 EC meter driver',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
