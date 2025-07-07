from setuptools import find_packages, setup

package_name = 'f1tenth_gap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/f1tenth_gap/launch', ['launch/follow_gap_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josue',
    maintainer_email='josue@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'follow_gap_node = f1tenth_gap.follow_gap_node:main',
        ],
    },
)
