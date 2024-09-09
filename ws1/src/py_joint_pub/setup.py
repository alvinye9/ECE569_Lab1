from setuptools import find_packages, setup

package_name = 'py_joint_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/lissajous.csv'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alvin Ye',
    maintainer_email='alvinye9@gmail.com',
    description='joint states publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = py_joint_pub.joint_publisher_test:main',
            'lissajous = py_joint_pub.joint_publisher_lissajous:main',
        ],
    },
)
