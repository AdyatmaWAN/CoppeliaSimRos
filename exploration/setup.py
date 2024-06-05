from setuptools import find_packages, setup

package_name = 'exploration'

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
    maintainer='alif',
    maintainer_email='alifwr98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = exploration.explore:main',
            'go = exploration.go:main',
            'input = exploration.input:main',
            'key = exploration.key:main',
        ],
    },
)
