from glob import glob
from setuptools import setup
setup(name='beni_sim', version='0.1.0', packages=['beni_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/beni_sim']),
        ('share/beni_sim', ['package.xml']),
        ('share/beni_sim/launch', glob('launch/*.launch.py')),
        ('share/beni_sim/models', glob('models/*.sdf')),
        ('share/beni_sim/worlds', glob('worlds/*.sdf')),
    ], install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Sumangowda Valagerepura Rajanna', maintainer_email='145328401+sumangowdavr@users.noreply.github.com',
    description='BENI-inspired assistant simulation', license='MIT',
    entry_points={'console_scripts': [
        'drive_guard = beni_sim.drive_guard:main',
        'follow_target = beni_sim.follow_target:main',
        'smoke_test = beni_sim.smoke_test:main',
    ]})
