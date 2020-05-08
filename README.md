![DeepClaw-Logo](doc/deepclawdoc/docs/asset/fig-DeepClaw.png)

# The DeepClaw Benchmark

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage for Julia can be found at [deepclaw.ancorasir.com](https://deepclaw.ancorasir.com/). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw, below.

## Resources

- Homepage: https://deepclaw.ancorasir.com/
- Documentation: https://bionicdl-sustech.github.io/DeepClawBenchmark/_build/html/index.html
- Paper explaining DeepClaw: [arXiv:2005.02588 [cs.RO]](https://arxiv.org/abs/2005.02588)
- Papers using DeepClaw: 
  - [arXiv:2003.01584 [cs.RO]](https://arxiv.org/abs/2003.01584)
  - [arXiv:2003.01583 [cs.RO]](https://arxiv.org/abs/2003.01583)
  - [arXiv:2003.01582 [cs.RO]](https://arxiv.org/abs/2003.01582)

## Code Organization

The DeepClaw code is organized as follows:

    configs/                    configuration for robotic station for manipulation tasks.
    deepclaw/drivers/           drivers for various robotic hardware, i.e. ur, franka, aubo.
    deepclaw/models/            model zoo for segmentation, classification, pick planning, and motion planning.
    deepclaw/utils/             server setup with dockers and client setup for laptops (x86) and jetson (arm).
    projects/proj_TrashSorting  a sample project to run deepclaw for sorting trash.
    datasets/trash              description of trash sorting dataset
    docs/                       description of this document as a manual.
    data/trash                  data on trash sorting

## Bibliography

```
@misc{wan2020deepclaw,
    title={DeepClaw: A Robotic Hardware Benchmarking Platform for Learning Object Manipulation},
    author={Fang Wan and Haokun Wang and Xiaobo Liu and Linhan Yang and Chaoyang Song},
    year={2020},
    eprint={2005.02588},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```