# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# !/usr/bin/python
# -*- coding:utf-8 -*-

from setuptools import setup, find_packages

setup(
    name='DeepClaw',
    version='1.0.1',
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
    url='https://bionicdl-sustech.github.io/DeepClawBenchmark/_build/html/index.html',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Software Development :: Libraries'
    ],
    install_requires=[
        'numpy>=1.18'
    ],
)