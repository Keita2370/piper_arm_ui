import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'piper_arm_ui'


def web_data_files():
    """web/ 以下をディレクトリ構造を保ったまま share/ にインストールする."""
    entries = []
    for root, _dirs, files in os.walk('web'):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test', 'tests', 'tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # .rviz も忘れずに入れる（yaml だけだと harvest.rviz が入らない）
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ] + web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keita',
    maintainer_email='hagihara516@gmail.com',
    description='PiPER アームロボット用ブラウザ操作UI (WebRTC + rosbridge)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_server = piper_arm_ui.ui_server:main',
            'rviz_bridge = piper_arm_ui.rviz_bridge:main',
            'harvest_sequencer = piper_arm_ui.harvest_sequencer:main',
            'dummy_perception = piper_arm_ui.dummy_perception:main',
        ],
    },
)
