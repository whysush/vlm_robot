import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vlm_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sushanth',
    maintainer_email='sudhanshyelishetty@gmail.com',
    description='Open-vocabulary, VLM-driven object-goal navigation: SLAM, Nav2, '
                'and a local vision-language model that finds named objects.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_goal_nav = vlm_navigation.object_goal_nav:main',
        ],
    },
)
