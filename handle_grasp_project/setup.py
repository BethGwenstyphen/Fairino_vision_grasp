from setuptools import setup
import os
from glob import glob

package_name = 'handle_grasp_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 【关键修正】强制系统把 config 文件夹里的 JSON 拷贝到安装目录
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Handle grasp vision and control nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册你的两个可执行程序
            'vision_node = handle_grasp_project.vision_node:main',
            'control_node = handle_grasp_project.control_node:main',
        ],
    },
)