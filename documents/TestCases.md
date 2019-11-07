# Test Cases

## Test the installation:

After installation, you can run following command to test your installation.

```shell
$ python main.py {ur5|ur10e} test
```

## Calibration

We provide a naive hand-eye calibrating method with RGB-D camera (RealSense D435) which utilizes 3-D dimension information. 

Before run this test case, you need to **rewrite parameters** in /Config/calibrating.yaml, such as "start point", "x_stride" and so on.

Then you can test the calibration:

```shell
$ python main.py {ur5|ur10e} calibration_test
```

