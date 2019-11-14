

class GraspPlaner(object):
    def display(self, color_image, depth_image, point_cloud,
                bounding_box, mask, centers,
                labels, probability):
        '''

        :param point_cloud:
        :param color_image: array of shape (width, height, 3)， optional
        :param depth_image: array of shape (width, height, 1), optional
        :param point_cloud: array of shape (number of points, 3), optional
        :param bounding_box:
        :param mask:
        :param centers:
        :param labels: array of shape (n_points, 1) or array of shape (width, height)
        :param probability: array of shape (n_points, n_labels) or array of shape (width, height, n_labels)

        :return: grasp_pose: list of length 6
        '''


class Custom(GraspPlaner):
    def __init__(self):
        raise NotImplementedError

    def display(self, **kwargs):
        raise NotImplementedError
