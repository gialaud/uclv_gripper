from setuptools import find_packages, setup

package_name = 'uclv_robotiq_gui'

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
    maintainer='mmirto',
    maintainer_email='michelemirto.mm@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'hand_e_gui = uclv_robotiq_gui.hand_e_gui:main', 
        ],
    },
)
