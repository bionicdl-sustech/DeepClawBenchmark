# Week 09 Simulated Robot Player

<img src="Tictactoe.jpeg" width = "300" height = "300"/>

In week 09 and 10, the students will apply their knowledge of robot simulation and computer vision to complete the project 2 of the course. The project aims to build a robot player of Tic-Tac-Toe in simulation.

The V-rep Scene file has provide in [here](../Simulation/scene/Tic_tac_toe.ttt), but for making robot move and play with you, you have to implement 3 basic elements:

- Computer Vision

    Try to recognize the object on the desk, classify the category of object and calculate the real world position of object via depth image and the previous result you got. It is essential information you should know if you want your virtual robot put the chess on the correct position. 

- The decision maker(Minmax or reinforcement learning method)

    The robot need to know where to put the chess that can lead it go to win. The classical method MINMAX is recommended and the reinforcement learning method that based on MDP is happily welcomed.

- Robot control

    Control robot to run on the trajectory that lead robot gripper grasp chess and release chess stable and robust.

Please go the [Simulation folder](../Simulation) for the detailed instructions. The following video is what you should have when complete the project.
![img](bpqcy-gr0tw.gif)