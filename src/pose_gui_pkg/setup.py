from setuptools import find_packages, setup

package_name = 'pose_gui_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='A simple GUI to publish goal poses.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line makes your script runnable with 'ros2 run'
            # 'executable_name = package_name.file_name:main_function'
            'pose_publisher = pose_gui_pkg.pose_publisher_gui:main',
        ],
    },
)
