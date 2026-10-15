from setuptools import find_packages, setup

package_name = 'diferencial_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime Landinez',
    maintainer_email='jaime.landinez@eia.edu.co',
    description='Cinemática de las llantas del robot diferencial y comparación con el /odom de Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cinematica_diferencial = diferencial_control.nodo_cinematica:main',
        ],
    },
)
