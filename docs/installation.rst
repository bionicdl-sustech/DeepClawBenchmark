Installation on Linux
=====================

This chapter describes how to install ``DeepClaw`` from source.
DeepClaw framework has only been tested with Python 2.7 and Ubuntu 16.04 LTS.

.. note::
   The following instructions are exemplary for Ubuntu 16.04 LTS system and `Python 2.7`.
   They only work in the supported environments.

Virtual Environment
-------------------
We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

Install virtualenv.

.. code-block:: shell
    :linenos:

    $ pip install -U virtualenv

Create a new virtual environment.

.. code-block:: shell
    :linenos:

    $ virtualenv -p /usr/bin/python2.7 ~/DCvenv

Activate or retreat from virtual environment.

.. code-block:: shell
    :linenos:

    $ source ~/DCvenv/bin/activate # activate virtual environment
    $ deactivate # retreat from virtual environment

Requirements
------------


Notes
-----
 * RG6 Driver: uncheck the ``Enable RG`` box under Installation/RG Configuration tab in UR5's teach pendent.
 
