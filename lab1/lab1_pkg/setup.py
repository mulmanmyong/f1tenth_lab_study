from setuptools import setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab1_launch.py']),  # Ensure this line is present
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeno',
    maintainer_email='mulmanmyong@gmail.com',
    description='lab1_pkg',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab1_pkg.talker:main',  # Ensure these match your executable names
            'relay = lab1_pkg.relay:main',
        ],
    },
)
