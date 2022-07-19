from setuptools import setup
import os 
from glob import glob

package_name = 'random_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='janisebastian100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_action = random_action.random_action:main',
        ],
    },
)
