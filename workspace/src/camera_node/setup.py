from setuptools import setup

package_name = 'camera_node'

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
    maintainer='bntti',
    maintainer_email='roysko.juho@gmail.com',
    description='Publishes images from the camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch = camera_node.camera_node:main',
        ],
    },
)
