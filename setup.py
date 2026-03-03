import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'IRB120'

mesh_files = []
for subdir in ('visual', 'collision'):
    source_pattern = os.path.join('urdf', 'meshes', subdir, '*')
    files = [f for f in glob(source_pattern) if os.path.isfile(f)]
    if files:
        mesh_files.append((f'share/{package_name}/urdf/meshes/{subdir}', files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/IRB120.urdf.xml']),
        ('share/' + package_name + '/urdf', ['urdf/camera.urdf.xml']),
        ('share/' + package_name + '/urdf', ['urdf/camera.sdf']),
        ('share/' + package_name + '/urdf', ['urdf/IRB120.rviz']),
        ('share/' + package_name + '/config', ['config/IRB120_controller.yaml']),
        ('share/' + package_name + '/moveit_config', glob('IRB120_moveit_config/config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + mesh_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='el7agadel',
    maintainer_email='adel0800@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = IRB120.state_publisher:main',
            'send_trajectory = IRB120.send_trajectory:main',
            'send_trajectory_moveit = IRB120.send_trajectory_moveit:main',
            'send_trajectory_pnp = IRB120.send_trajectory_pnp:main',
            'camera_viewer = IRB120.camera_viewer:main',
        ],
    },
)
