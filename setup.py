from setuptools import find_packages, setup

package_name = 'dynamixel_controller_client'

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
    maintainer='kaoru',
    maintainer_email='kaoru.yoshida@keio.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_controller_client_node = dynamixel_controller_client.dynamixel_controller_client_node:main'
        ],
    },
)
