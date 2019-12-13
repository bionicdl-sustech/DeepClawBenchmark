# How to Create New Tasks

This is a guidance for how to create a new manipulation task by using DeepClaw benchmark. We encorage developers to create their own tasks following this guidance and welcome to release issues here.

- [Directory of Tasks](#directory-of-tasks)
- [Basic Structure of Manipulation Tasks](#basic-structure-of-manipulation-tasks)
- [Modules in Task](#modules-in-task)
- [Data Conservation](#data-conservation)

## <a name="directory-of-tasks">Directory of Tasks</a>

All tasks code save in DeepClaw_root_path/examples, like that

```shell
├─examples
│      __init__.py

│      Calibration.py
│      JigsawPuzzle.py
│      ToyClawMachine.py

│      Task.py
│      TestCase.py
|      ...
```

Please put all tasks files here.

## <a name="basic-structure-of-manipulation-tasks">Basic Structure of Manipulation Tasks</a>

We standardized all manipulation tasks into five stages: segmentation, recognition, grasp plannng, motion planning and execution. Even the last two stages are always implemented in the controller provided by manufacturer for most robot arms, we hold them in the basic structure. 

The basic structure of NewTask.py is as follows:

```python
# import algorithm modules
# from modules.localization.algorithm_A import algorithm_A
# from modules.recognition.algorithm_B import algorithm_B
# from modules.grap_planning.algorithm_C import algorithm_C
# from modules.motion_planning.algorithm_D import algorithm_D
# from modules.execution.algorithm_E import algorithm_E

class NewTask(Task):
    def __init__(self, args):
    # section 1
    # define specifical arguments
    # and initial monitor and publisher instantiations

    def task_display(self):
    # section 2
    # always be loop here to repeat subtask several times

    def subtask_display(self):
    # section 3
    # in suggestion, performe functions step by step here
    # for example
        # output_a = self.localization_operator.display(input_a)
        # output_b = self.recognition_operator.display(input_b)
        # output_c = self.grasp_planner.display(input_c)
        # output_d = self.motion_planner.display(input_d)
        # output_e = self.execution.display(input_e)

# other typical referencec here
# def function_a():
#     return xxx
```

## <a name="modules-in-task">Modules in Task</a>

Please check [here](https://github.com/bionicdl-sustech/DeepClawBenchmark/tree/python2.7/modules) to find all modules can be used in each stage in task.

> Modules implement standardized I/O information between stages, thus developer can substitute them without doing any extra work.

## <a name="data-conservation">Data Conservation</a>

DeepClaw provides **a monitoring system** for users to record information during experiment. This system is established based on *Observer Pattern*, shows as following.

(observer pattern)

All monitors and publisher locate in *input_output* folder, like that

```shell
├─input_output
│  │  ...
│  │  Monitor.py
│  │  __init__.py
│  │  
│  ├─observers
│  │      AbstractObserver.py
│  │      ImageMonitor.py
│  │      PCLMonitor.py
│  │      TimeMonitor.py
│  │      __init__.py
│  │      
│  └─publishers
│          AbstractSubject.py
│          Publisher.py
│          __init__.py
│  ...
```

Before users using their monitors to record data, all used monitors should be registered by the pulisher at section 1 in above pseudocode, like following.

```python
 # ...
 self.publisher = Publisher("publisher")
 self.time_monitor = TimeMonitor("time_monitor")

 self.image_monitor = ImageMonitor("image_monitor")

 self.data_monitor = DataMonitor("data_monitor")

 self.publisher.registerObserver(self.time_monitor)

 self.publisher.registerObserver(self.image_monitor)

 self.publisher.registerObserver(self.data_monitor)
 # ...
```

Then, using publisher to broadcast data when requiring it during task displaying. For example:

```python
rgb_information = {"Image", color_image}
self.publisher.sendData(rgb_information)
```

Information type used here is **dict** in Python, the key is dtermined by the monitor who capture this information. DeepClaw provides basic monitors such as Image Monitor, Time Monitor and so on. It is better to create a specifical monitor for task. The Image Monitor looks like this:

```python
class ImageMonitor(AbstractObserver):
    def __init__(self, name):
        self.name = name
        self.data = {}
        self.dir = ''
        self.img_name = ''

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        # if self.data.has_key('Image'):
        if "Image" in self.data:
            if not os.path.exists(self.dir):
                os.makedirs(self.dir)
            cv2.imwrite(self.dir + self.img_name, self.data['Image'])
```

One more thing, to unify data saving path, we recommand that using following code in section 1 in abve task code templete, to assignment saving path automaticlly.

```
time_s = time.localtime(int(time.time()))
self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
self.data_path = _root_path+"/data/"+os.path.basename(__file__).split(".")[0]+"/"+self.experiment_name+"/"
```

All the data will be saved in *DeepClaw_root_path/data/*，and an example is like that (if new task file name is "TicTacToe"):

```shell
├─TicTacToe

│  └─experiment_xxxxx

│      │  experiment_xxxxx_CostTime.csv
│      │  experiment_xxxxx_TicTacToeData.csv
│      │              
│      ├─ subtask_0
│      ├─ subtask_1
│      ├─ subtask_2
│      ├─ subtask_3
│      ├─ ...
```
