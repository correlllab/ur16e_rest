from setuptools import find_packages, setup

package_name = 'ur16e_rest'

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
    maintainer='max',
    maintainer_email='maxlconway@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur16e_rest_api_node = ur16e_rest.scripts.node:main',
            'orchestrator_node = ur16e_rest.scripts.orchestrator_node:main',
            'test_node = ur16e_rest.scripts.test_node:main',
        ],
    },
)
