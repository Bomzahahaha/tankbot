from setuptools import find_packages, setup

package_name = 'median_filter'

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
    maintainer='ubuntu',
    maintainer_email='komkanin.m@gmail.com',
    description='Median filtering and weld seam detection nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'median = median_filter.median_filter:main',
            'improved_median = median_filter.improved_median_filter:main',
            'dynamic_median = median_filter.dynamic_median:main',
            'weld_detector_median = median_filter.weld_detector_median:main',
        ],
    },
)