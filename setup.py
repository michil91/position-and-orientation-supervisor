from setuptools import setup, find_packages

package_name = 'poise'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='POISE Maintainer',
    maintainer_email='maintainer@example.com',
    description='Position and Orientation Integrity Supervision Engine',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_publisher = poise.sim.gnss_publisher:main',
            'imu_publisher = poise.sim.imu_publisher:main',
            'gnss_imu_checker = poise.checks.gnss_imu_checker:main',
            'integrity_aggregator = poise.core.integrity_aggregator:main',
        ],
    },
)
