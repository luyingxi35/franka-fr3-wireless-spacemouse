from setuptools import find_packages, setup

package_name = 'spacemouse_publisher'

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
    maintainer='root',
    maintainer_email='support@franka.de',
    description='This package provides a node that publishes the data from a 3Dconnexion SpaceMouse.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyspacemouse_publisher = spacemouse_publisher.pyspacemouse_publisher:main',
        ],
    },
)
