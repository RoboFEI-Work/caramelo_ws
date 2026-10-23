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
        (
            'share/' + package_name + '/models/pt_BR-faber',
            glob('models/pt_BR-faber/*'),
        ),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Equipe RoboFEI @Work',
    maintainer_email='robofei.atwork@gmail.com',
    description='Voz do robo: fila com prioridade, cache e backends Kokoro/Piper/espeak.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'speech_node = manip_audio.speech_node:main',
        ],
    },
)
