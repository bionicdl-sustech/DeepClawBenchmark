# Module Pool

DeepClaw maintains a module pool of algorithms and end-to-end robot learning models by integrating some state-of-the-art research results in computer vision and robotics. The codes are placed under deepclaw/modules/.

## Computation on server
As the running environment for each method is different, DeepClaw adopts concepts from cloud robotics. We put the running environments for end-to-end methods which requires heavy computations in a docker container on the server, and deploy the robot control and basic computations on a user computer.

Currently we have two servers: Goldenboy and Serbreeze. Currently they are running Ubuntu16.04 and cuda9.0. We plan to upgrade them to Ubuntu18.04 and cuda10 soon.

Each user is assigned to have one GPU card by setting environment varible CUDA_VISIBLE_DEVICES. Please don't change it by yourself. If you need more computation resources, please contact us.
   
|           | Goldenby                                     | Serbreeze                                         |
|-----------|----------------------------------------------|---------------------------------------------------|
| Memory    | 251.8G                                       |    125.8 GiB                                      |
| Processor | Intel® Xeon(R) CPU E5-2698 v4 @ 2.20GHz × 40 | Intel® Xeon(R) CPU E5-2650 v4 @ 2.20GHz × 48      |
| GPU       | Tesla V100 32G x4                            | GeForce GTX 1080Ti 12G x4                         |
| Storage   | 7.6TB SSD                                    | 240G SSD (/home), 960GB SSD+8TB HD (/media/amax/) |
| Users     | Standard: user-1, user-2, user-3             | Standard: student1, student2, student3            |
| IP        | 10.20.123.35                                 | 10.20.73.134                                      |


## List of modules
### Segmentation
| Method          | Object classes    | weights                                                                    |
|-----------------|-------------------|----------------------------------------------------------------------------|
| Contour detector| NA                | NA |

### Recognition

### Object Detection
| Method       | Object classes    | weights                                                                    |
|--------------|-------------------|----------------------------------------------------------------------------|
| Efficientdet | 204 waste classes | [link](https://pan.baidu.com/s/1GiQSp-fWK_711mn13MPXow) extract code: frra |
 