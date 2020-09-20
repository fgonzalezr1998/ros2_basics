from setuptools import setup

package_name = 'reconf_params'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Fernando Gonzalez',
    author_email='fergonzaramos@yahoo.es',
    maintainer='Fernando Gonzalez',
    maintainer_email='fergonzaramos@yahoo.es',
    description='yolact_ros2 reconfigurable paameters client',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reconf_params_node = reconf_params.reconf_params_node:main',
        ],
    },
)
