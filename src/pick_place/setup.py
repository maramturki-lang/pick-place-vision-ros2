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
        ('share/' + package_name + '/models',
            ['models/yolov8n_pick_place.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maram',
    maintainer_email='maram.turki@etudiant-enit.utm.tn',
    description='Pick and place guide par vision',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'move_to_pose = pick_place.move_to_pose:main',
            'spawn_scene = pick_place.spawn_scene:main',
            'project_check = pick_place.project_check:main',
            'generate_dataset = pick_place.generate_dataset:main',
            'detector = pick_place.detector:main',
        ],
    },
)
