from setuptools import find_packages, setup

package_name = 'practicaR1'

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
    maintainer='laboratorio',
    maintainer_email='inigo.fiat@opendeusto.es',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'practica1 = practicaR1.practica1:main',
            'trayectorias = practicaR1.trayectorias:main_trayectorias',
            'grabador = practicaR1.parte1:main_grabador',
            'practica = practicaR1.parte1:main_practica',
            'grabador2 = practicaR1.parte2:main_grabador_inteligente',
            'practica2 = practicaR1.parte2:main_parte2',
        ],
    },
)
