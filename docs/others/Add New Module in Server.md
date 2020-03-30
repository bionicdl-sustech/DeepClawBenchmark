# Add New Module in Server
#### Login
Login using assigned account

```shell
$ ssh student#@192.168.1.104
```

#### Create a new docker container from DeepClaw image
Create a new container using image **deepclaw:v1**.
```shell
$ docker run -p 2020:2020 -itd --name <your_docker_image_name> deepclaw:v1 /bin/bash
```
where `-p` is used to blind host's port and container's port, we blind 2020 port here.

List all containers who are running now.

```shell
$ docker ps
```
You can check the ID of container in the first column, and you can find the ID of your container by checking the container's name in the last column.

Then you can enter into your container by the command `docker exec`.

```shell
$ docker exec -it <your_container_ID> /bin/bash
```
The benefit of `docker exec` is that the container won't stop when you `exit` your container.
#### Get DeepClaw
After entering your container, you should `cd` to the path `/home`, and then clone the DeepClaw.
```
root@<your_container_ID>:/# cd /home
root@<your_container_ID>:/home git clone https://github.com/ancorasir/DeepClaw.git
root@<your_container_ID>:/home cd DeepClaw
```
#### Create a new algorithm module
All the algorithm modules of DeepClaw are stored in /modules`, you can find details of DeepClaw structure here.

We can create a new algorithm by the copy of the simple template `HelloDC.py`

```shell
root@<your_container_ID>:/home/DeepClaw cd modules
root@<your_container_ID>:/home/DeepClaw/modules cp HelloDC.py segmentation/<your_algorithm_name>
```
Please note that your algorithm's name has `.py` postfix.
We can check the code in `HelloDC.py` by vim

```
root@<your_container_ID>:/home/DeepClaw/modules vim segmentation/<your_algorithm_name>
```
```python
# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: 
@Author: 
@Date: 
@Description:
"""
# import package here

class YourAlogrithmName(object):
    def __init__(self, arg1, arg2, argn):
        # initialization
        pass

    def run(self, input_data):
        # algorithm implementation here
        pass

# static functions here

```

If you are not familiar with vim, you can write your code in local, then upload them into the server.

#### Start server

We can start the algorithm server in python command line.

```shell
root@<your_container_ID>:/home/DeepClaw python
Python 3.8.2 (default, xxxx, xxxxx)
[GC 8.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>>from utils.Server import Server
>>>s=Server('0.0.0.0', 2020)
>>>s.run()
Server starting...
```
