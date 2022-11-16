# SPDX-FileCopyrightText: 2022-2023, G.A. vd. Hoorn
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup

package_name = 'motoros2_fjt_pt_queue_proxy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='G.A. vd. Hoorn',
    author_email='g.a.vanderhoorn@tudelft.nl',
    maintainer='G.A. vd. Hoorn',
    maintainer_email='g.a.vanderhoorn@tudelft.nl',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Drip-feed FJT goals to MotoROS2 one traj pt at a time',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'server = ' + package_name + '.server:main',
        ],
    },
)
