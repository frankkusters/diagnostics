from setuptools import setup

package_name = 'diagnostic_analysis'

setup(
    name=package_name,
    version='1.9.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Austin Hendrix',
    maintainer_email='namniart@gmail.com',
    description='The diagnostic_analysis package can convert a log of diagnostics data '
        'into a series of CSV files. Robot logs are recorded with rosbag, and '
        'can be processed offline using the scripts in this package.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exporter = diagnostic_analysis.scripts.export_csv:main',
        ],
    },
)
