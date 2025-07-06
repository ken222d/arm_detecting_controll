from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'arm_detecting_controll'

def generate_data_files(share_path, dir):
   data_files = []
   for path, _, files in os.walk(dir):
       list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
       data_files.append(list_entry)
   return data_files


setup(
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
       ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
       (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
       (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
       (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
       (os.path.join('share', package_name, 'world'), glob('world/*')),
   ] + generate_data_files('share/' + package_name + '/', 'models'),
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='Kenta Ishizeki',
   maintainer_email='a.w.g.d0201@icloud.com',
   description='catch the ball with arm',
   license='BSD-3-Clause',
   tests_require=['pytest'],
   entry_points={
       'console_scripts': [
           'multi_color_ball_detector = arm_detecting_controll.multi_color_ball_detector:main',
           'ball_follower = arm_detecting_controll.ball_follower:main',
           'arm_controll = arm_detecting_controll.arm_controll:main',
           'attach_arm_ball = arm_detecting_controll.attach_arm_ball:main',
           'linetrace_controller = arm_detecting_controll.linetrace_controller:main',
           'linetrace_judgement = arm_detecting_controll.linetrace_judgement:main',
           'teleop_motor = arm_detecting_controll.teleop_motor:main',
        ],
   },
)
