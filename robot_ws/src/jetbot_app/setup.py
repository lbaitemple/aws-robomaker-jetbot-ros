from setuptools import setup, find_packages
from glob import glob

PACKAGE_NAME = 'jetbot_app'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*')),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717',
    maintainer_email='animesh.ani@live.com',
    description='Jetbot Simulation App Package for Robot Running',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle = jetbot_app.circle:main',
            'teleop = jetbot_app.teleop:main',
            'move = jetbot_app.adafruit_move:main',
        ],
    },
)
