from setuptools import setup

package_name = 'path_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gin',
    maintainer_email='ean18016@student.mdu.se',
    description='Path follower follows a predefined path made up of waypoints',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_follower = path_follower.path_follower:main',
            'demo_path_traversal = path_follower.demo_path_traversal:main',
        ],
    },
)
