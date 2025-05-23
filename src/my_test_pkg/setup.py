from setuptools import find_packages, setup

package_name = 'my_test_pkg'

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
    maintainer='rajan',
    maintainer_email='rajankhade31@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_camera_images = my_test_pkg.process_camera_images:main',
            'pose_estimation = my_test_pkg.pose_estimation:main',
            'navigation_01 = my_test_pkg.navigation_01:main',
            'navigation_02 = my_test_pkg.navigation_02:main',
            'navigation_03 = my_test_pkg.navigation_03:main',
        ],
    },
)
