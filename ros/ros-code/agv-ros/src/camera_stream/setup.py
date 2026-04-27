from setuptools import setup

package_name = 'camera_stream'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'stream_node = camera_stream.stream_node:main',
        ],
    },
)