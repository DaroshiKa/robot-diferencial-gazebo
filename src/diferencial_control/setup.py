from setuptools import find_packages, setup

package_name = 'diferencial_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime Landinez',
    maintainer_email='jaime.landinez@eia.edu.co',
    description='Cinemática, AEB por TTC, seguimiento de pared (PID), mux de comandos y cronómetro de vueltas',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cinematica_diferencial = diferencial_control.nodo_cinematica:main',
            'mux_cmd = diferencial_control.nodo_mux:main',
            'aeb = diferencial_control.nodo_aeb:main',
            'seguidor_pared = diferencial_control.nodo_seguidor_pared:main',
            'vueltas = diferencial_control.nodo_vueltas:main',
            'analizar_odometria = diferencial_control.analizar_odometria:main',
        ],
    },
)
