from setuptools import find_packages, setup

package_name = 'mi_odometria'

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
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'leer_orientacion = mi_odometria.leer_orientacion:main',
        	'control_orientacion = mi_odometria.control_orientacion:main',
        	'lineaRecta_odom = mi_odometria.lineaRecta_odom:main',
        	'lineaRecta_tiempo = mi_odometria.lineaRecta_tiempo:main',
        	'lineaRecta_serviceServer = mi_odometria.lineaRecta_serviceServer:main',
        	'lineaRecta_serviceClient = mi_odometria.lineaRecta_serviceClient:main',
            'movimiento_serviceServer = mi_odometria.movimiento_serviceServer:main',
            'movimiento_serviceClient = mi_odometria.movimiento_serviceClient:main',
            'movimiento_actionServer = mi_odometria.movimiento_actionServer:main',
            'movimiento_actionClient = mi_odometria.movimiento_actionClient:main',
            'triangulo_actionServer = mi_odometria.triangulo_actionServer:main',
            'triangulo_actionClient = mi_odometria.triangulo_actionClient:main',
            'poligono_actionServer = mi_odometria.poligono_actionServer:main',
            'poligono_actionClient = mi_odometria.poligono_actionClient:main',
            'movimientoLaser_actionServer = mi_odometria.movimientoLaser_actionServer:main',
            'movimientoLaser_actionClient = mi_odometria.movimientoLaser_actionClient:main'
        ],
    },
)
