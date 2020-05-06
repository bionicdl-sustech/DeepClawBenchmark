![](docs/asset/fig-DeepClaw.png)

# The DeepClaw Benchmark

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage for Julia can be found at [deepclaw.ancorasir.com](https://deepclaw.ancorasir.com/). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw, below.

# Resources

- Homepage: https://deepclaw.ancorasir.com/
- Documentation: https://bionicdl-sustech.github.io/DeepClawBenchmark/_build/html/index.html
- Paper explaining DeepClaw: arXiv ([BibTex](#bibliography))
- Papers using DeepClaw: 
  - arXiv
  - arXiv
  - arXiv

# Installation

XXXX

# Code Organization

The DeepClaw code is organized as follows:

    configs/                configuration for robotic station for manipulation tasks.
    deepclaw/drivers/       drivers for various robotic hardware, i.e. ur, franka, aubo.
    deepclaw/models/        model zoo for segmentation, classification, pick planning, and motion planning.
    deepclaw/utils/         server setup with dockers and client setup for laptops (x86) and jetson (arm).
    projects/proj_claw      a sample project to run deepclaw in arcade claw game.
    projects/proj_jigsaw    a sample project to run deepclaw in jigsaw game.
    projects/proj_tictactoe a sample project to run deepclaw in tic-tac-toe game.
    datasets/toys           description of the toy dataset
    datasets/jigsaw         description of jigsaw game pieces dataset
    datasets/trash          description of trash sorting dataset
    datasets/mnist          description of mnist dataset
    docs/                   description of this document as a manual.

# Bibliography

arXiv