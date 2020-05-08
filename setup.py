# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# !/usr/bin/python
# -*- coding:utf-8 -*-

from setuptools import setup, find_packages

setup(
    name='DeepClaw',
    version='1.0.3',
    description=(
        'a reconfigurable benchmark of robotic hardware and task hierarchy for robot learning'
    ),
    author='BionicDL',
    author_email='sirgroup@outlook.com',
    maintainer='Haokun W., Fang W., Xiaobo L., Yanglin H.',
    maintainer_email='wanghk@mail.sustech.edu.cn',
    license='MIT License',
    packages=find_packages(),
    platforms=["all"],
    url='https://bionicdl-sustech.github.io/DeepClawBenchmark/',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Software Development :: Libraries'
    ],
    install_requires=[
        'numpy==1.18.2',
        'matplotlib==3.2.1',
        'opencv_contrib_python==4.1.2.30',
        'PyYAML==5.3.1',
        'pyrealsense2==2.34.0.1470',
        'scipy==1.4.1',
        'tqdm==4.46.0'
    ],
)
