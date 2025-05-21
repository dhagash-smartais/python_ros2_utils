from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhagash Desai',
    maintainer_email='dhagash.desai@smartais.de',
    description='This is package that contains most used notes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_rotator = utils.cloud_rotator:main'
        ],
    },
)
