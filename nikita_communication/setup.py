from setuptools import setup
import os, glob

package_name = 'nikita_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],

    data_files=[
        ('lib/' + package_name, ['nikita_communication/chatbot.py', 'nikita_communication/music_player.py', 'nikita_communication/stoppable_thread.py', 'nikita_communication/stt_offline.py', 'nikita_communication/stt_online.py', 'nikita_communication/tts.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/communication_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/params.yaml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # ('target_directory_2', glob('nested_source_dir/**/*', recursive=True))
    ],
    install_requires=[
        'setuptools',
        ],
    zip_safe=True,
    maintainer='Christian Stein',
    maintainer_email='stein.robotics@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_communication = nikita_communication.node_communication:main',
        ],
    },
)
