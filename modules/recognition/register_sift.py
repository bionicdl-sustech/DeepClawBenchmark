

from recognition import Recognition

class Recognition(object):
    def display(self, color_image, depth_image, point_cloud, bounding_box, mask, centers):
        '''

        :param color_image: array of shape (width, height, 3)， optional
        :param depth_image: array of shape (width, height, 1), optional
        :param point_cloud: array of shape (number of points, 3), optional
        :param bounding_box: array of shape (2, 2) or array of shape (3, 3), optional
        :param mask: array of shape (n_points, 1) or array of shape (width, height), optional

        :return: labels: array of shape (n_points, 1) or array of shape (width, height)
                 probabilities: array of shape (n_points, n_labels) or array of shape (width, height, n_labels)
        '''


class Custom(Recognition):
    def __init__(self):
        raise NotImplementedError

    def display(self, bounding_box, mask, centers, **kwargs):
        raise NotImplementedError
