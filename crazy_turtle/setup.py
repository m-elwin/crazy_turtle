from setuptools import setup

package_name = 'crazy_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/go_crazy_turtle.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Drive a turtle on a crazy path',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover = crazy_turtle.mover:main'
        ],
    },
)
