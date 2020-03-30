Example: Tic-Tac-Toe
====================

.. figure:: _static/fig-TicTacToePipeline.png
    :align: center
    :figclass: align-center


Tasks Description
-----------------
 Tic-Tac-Toe game is a temporal reasoning related task, which required two players moving pieces alternately.
 To simplify this game as a baseline, the two players use the same placing strategy, namely the Minimax algorithm with depth 3, and are both executed by the robot arm..

Configuration
--------------
 #. Manipulation Environment Description
      * A table top in front of the robot base
      * Camera mounted about 1 meter above the table top and looking downward.
      * gripper always kept vertically.

 #. Objects Information
      * Green and blue cubes from YCB objects set as two types of pieces
      * At the start of the task, checkerboard is placed on the middle of table top and pieces are place on two sides of the checkerboard

 #. Robots/Hardware Setup
      * UR10e
      * HandE

Metrics
-------
 Time cost of each functionality step and whether the game can be completed with an end.
