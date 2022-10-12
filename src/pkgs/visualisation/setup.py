from setuptools import setup

package_name = 'visualisation'

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
    description='Visualisation of sensor values',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_visualisation = visualisation.data_visualisation:main'
        ],
    },
)
