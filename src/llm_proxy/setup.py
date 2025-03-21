from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'llm_proxy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', '.env']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'python-dotenv', 'openai'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='2194521087@qq.com',
    description='ROS2 proxy for LLM requests and response analysis',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_proxy_node = llm_proxy.llm_proxy_node:main',
            'test_client_node = llm_proxy.test_client_node:main',
        ],
    },
)
