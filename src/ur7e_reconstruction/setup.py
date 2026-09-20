from glob import glob
from setuptools import find_packages, setup

setup(
    name='ur7e_reconstruction', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur7e_reconstruction']),
        ('share/ur7e_reconstruction', ['package.xml', 'README.md', 'TEST_RESULTS.md']),
        ('share/ur7e_reconstruction/config', glob('config/*.yaml')),
        ('share/ur7e_reconstruction/launch', glob('launch/*.launch.py')),
        ('share/ur7e_reconstruction/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=False,
    maintainer='wqj', maintainer_email='wqj@example.com', license='Apache-2.0',
    description='Eye-in-hand TSDF surface reconstruction without ESDF updates.',
    entry_points={'console_scripts': [
        'evaluate_scene = ur7e_reconstruction.evaluate:main',
        'scan_scene = ur7e_reconstruction.scan:main',
        'reconstruct = ur7e_reconstruction.node:main',
        'verify_reconstruction = ur7e_reconstruction.verify:main',
    ]},
)
