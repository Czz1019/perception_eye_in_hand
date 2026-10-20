from glob import glob
from setuptools import find_packages, setup

setup(
    name='ur7e_nvblox', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur7e_nvblox']),
        ('share/ur7e_nvblox', ['package.xml', 'README.md', 'TEST_RESULTS.md']),
        ('share/ur7e_nvblox/config', glob('config/*.yaml')),
        ('share/ur7e_nvblox/launch', glob('launch/*.launch.py')),
        ('share/ur7e_nvblox/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'], zip_safe=False,
    maintainer='wqj', maintainer_email='wqj@example.com', license='Apache-2.0',
    description='TF-based eye-in-hand ESDF mapping using realtime_nvblox.',
    entry_points={'console_scripts': [
        'tf_mapper = ur7e_nvblox.node:main',
        'verify_esdf = ur7e_nvblox.verify:main',
    ]},
)
