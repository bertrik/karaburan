from glob import glob

from setuptools import find_packages, setup


package_name = 'karaburan_navigation_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.sdf')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='mleegwt@users.noreply.github.com',
    description='Repeatable navigation system tests for Karaburan.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maneuver_test_runner = '
            'karaburan_navigation_tests.maneuver_test_runner:main',
            'maneuver_test_report = '
            'karaburan_navigation_tests.maneuver_report:main',
            'junit_html_report = '
            'karaburan_navigation_tests.junit_html_report:main',
        ],
    },
)
