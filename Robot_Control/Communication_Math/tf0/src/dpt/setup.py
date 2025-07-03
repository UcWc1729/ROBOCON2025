from setuptools import find_packages, setup

package_name = 'dpt'

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
    maintainer='ljhua',
    maintainer_email='ljhua@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        ['rpn = dpt.send_test:main',
         'stb=dpt.stb:main',
         'qj=dpt.qjhs:main'
        ],
    },
)
