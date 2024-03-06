from setuptools import find_packages, setup

package_name = 'v2x_light'

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
    maintainer='RogerJung',
    maintainer_email='engineer54rj@gmail.com',
    description='This package is about Autoware traffic light control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v2x_light = v2x_light.v2x_light:main'
        ],
    },
)
