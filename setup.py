from setuptools import setup

package_name = 'traffic_light_dataset_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/traffic_light_dataset_publisher.launch.xml']),
        ('share/' + package_name + '/config',
         ['config/test_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daisuke',
    maintainer_email='border_goldenmarket@yahoo.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            package_name + ' = ' + package_name + '.dataset_publisher:main',
        ]
    }
)
