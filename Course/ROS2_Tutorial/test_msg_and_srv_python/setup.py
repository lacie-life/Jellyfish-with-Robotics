from setuptools import setup

package_name = 'test_msg_and_srv_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='life',
    maintainer_email='00sao00ios00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = test_msg_and_srv_python.pub:main',
            'sub = test_msg_and_srv_python.sub:main',
        ],
    },
)
