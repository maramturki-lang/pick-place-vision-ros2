from setuptools import find_packages, setup

package_name = 'pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maram',
    maintainer_email='maram.turki@etudiant-enit.utm.tn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_pose = pick_place.move_to_pose:main',
            'spawn_scene = pick_place.spawn_scene:main',
            'project_check = pick_place.project_check:main',
        ],
    },
)
