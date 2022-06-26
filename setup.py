from setuptools import setup

package_name = 'camera_projector_alignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='tim.fanselow@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = camera_projector_alignment.calibration:main',
            'projector_align = camera_projector_alignment.projector_align:main',
            'project_grid = camera_projector_alignment.project_grid:main',

        ],
    },
)
