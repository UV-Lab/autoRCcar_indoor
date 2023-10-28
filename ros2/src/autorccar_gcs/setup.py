from setuptools import find_packages, setup

package_name = 'autorccar_gcs'

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
    maintainer='luke7637',
    maintainer_email='luke7637@gamil.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autorccar_gcs = autorccar_gcs.pyqt_gcs_with_pyqtgraph:main'
        ],
    },
)
