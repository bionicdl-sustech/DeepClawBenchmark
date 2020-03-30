# ME336-Wk07-Lab: TensorFlow Basic Tutorials
In today's lab session, we will learn some tensorflow basic operations and run some examples.

- [TensorFlow Basics](https://www.guru99.com/tensor-tensorflow.html): Tensor, Shape, Type, Graph, Sessions & Operators
- Tutorials: the examples are handwritten digits classification and image classification. The datasets used is [MNIST]((http://yann.lecun.com/exdb/mnist/)) and [FASHION MNIST](https://github.com/zalandoresearch/fashion-mnist#get-the-data), and they have been downloaded as mnist.npz and Fashion-MNIST in week07 folder.

## TensorFlow Installation
As [TensorFlow](https://pytorch.org/get-started/locally/#windows-anaconda)=2.x is not available from conda, we use pip to install it.
> ``pip install tensorflow==2.0``
> #if it's too slow, use other source like below    
> ``pip install tensorflow==2.0 -ihttps://pypi.tuna.tsinghua.edu.cn/simple/``

## TensorFlow Basic
Follow this [page](https://www.guru99.com/tensor-tensorflow.html) for a qucik overview of tensorflow basics. In this tutorial, you will learn:
- [What is a Tensor?](https://www.guru99.com/tensor-tensorflow.html#1)
- [Representation of a Tensor](https://www.guru99.com/tensor-tensorflow.html#2)
- [Types of Tensor](https://www.guru99.com/tensor-tensorflow.html#3)
- [Create a tensor of n-dimension](https://www.guru99.com/tensor-tensorflow.html#4)
- [Shape of tensor](https://www.guru99.com/tensor-tensorflow.html#5)
- [Type of data](https://www.guru99.com/tensor-tensorflow.html#6)
- [Creating operator](https://www.guru99.com/tensor-tensorflow.html#7)
- [Some Useful TensorFlow operators](https://www.guru99.com/tensor-tensorflow.html#8)
- [Variables](https://www.guru99.com/tensor-tensorflow.html#9)
- [Placeholder](https://www.guru99.com/tensor-tensorflow.html#10)
- [Session](https://www.guru99.com/tensor-tensorflow.html#11)
- [Graph](https://www.guru99.com/tensor-tensorflow.html#12)

## TensorFlow Tutorials
The official tutorial of tensorflow is [here](https://www.tensorflow.org/tutorials). And we also get some quick starts:
- **[mnist](http://yann.lecun.com/exdb/mnist/)**: run the [mnist.ipynb](./mnist.ipynbb). You can also refer to this [link](https://www.tensorflow.org/tutorials/quickstart/beginner).
- **[Fashion_mnist](https://github.com/zalandoresearch/fashion-mnist#get-the-data)**: Install matplotlib first as we need show figures ``conda install -c conda-forge matplotlib``; then run the [Fashion_mnist.ipynb](./Fashion_mnist.ipynb). You can also refer to this [link](https://www.tensorflow.org/tutorials/keras/classification).
