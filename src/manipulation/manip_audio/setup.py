from glob import glob

from setuptools import find_packages, setup


package_name = 'manip_audio'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/models/pt_BR-jeff',
            glob('models/pt_BR-jeff/*'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='RoboFEI Work',
    maintainer_email='robofei-work@todo.todo',
    description='Sintese de voz desacoplada para o manipulador.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'speech_node = manip_audio.speech_node:main',
        ],
    },
)
