from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
            # executable_name = package_name.module_name:main
            # executable_name not need to be same as node name
            'py_node = my_py_pkg.my_first_node:main',
            'py_node2 = my_py_pkg.my_first_node_oop:main',
            'py_node_timer = my_py_pkg.my_first_node_timer:main',
        ],
    },
)
