# data

All kinds of public datasets for benchamrking purposes.

# MNIST
The MNIST database is a handwritten digits data set, has a training set of 60,000 examples, and a test set of 10,000 examples. The digits have been size-normalized and centered in a fixed-size image (28x28). There are 4 files in the dateset: training set images, training set labels, test set images, and test set labels. See more details and download in [here](http://yann.lecun.com/exdb/mnist/).


# Fashion-MNIST
 Fashion-MNIST is a fashion products data set from 10 categories,  consists of a training set of 60,000 examples and a test set of 10,000 examples. Go to the [github page](https://github.com/zalandoresearch/fashion-mnist#get-the-data) for more detailed information and downloading data set.

# CIFAR
The CIFAR-10 and CIFAR-100 are labeled subsets of the 80 million tiny images dataset. They were collected by Alex Krizhevsky, Vinod Nair, and Geoffrey Hinton. See more details and download in [here](https://www.cs.toronto.edu/~kriz/cifar.html).



# Haihua-Trash-Sorting
The Dataset is provided by the 2020 Haihua AI Challenge·Garbage Classification. Please visit the [website](https://www.biendata.com/competition/haihua_wastesorting_task2/data/) for more information on the competition.

The original training dataset provides 80,000 images containing a single type of trash in each image (simple data) and 2998 images containing multiple types of trashes (up to 20 types) in each image (complex data). Each image is 1920x1080 in size. The labels provide the bounding boxes and the corresponding classification labels. Beside, the competition also provides 10000 simple images and 1000 complex images containing multiple types of trashes for testing without labels. There are 204 classes of trashes in total.

Due to the size of the original dataset (180G simple data and 18G complex data), it is quite challenge to download the full dataset. Hence we extract part of the simple dataset which belong to the following four recyclable material: glass, paper, metal and plastics. The recyclable dataset contains 14122 images of a single trash in each image. Images are resized to 960x540, which is half of the original resolution. There are four categories and the original category id is kept as category_id_ori in the label file.

# To be added
