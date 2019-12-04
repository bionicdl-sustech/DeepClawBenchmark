

class Localization(object):
    def display(self, color_image, depth_image, point_cloud):
        '''

        :param color_image: array of shape (width, height, 3), RGB image array, sampled from RGB camera sensor.
        :param depth_image: array of shape (width, height, 1), Depth image array, provide depth information at each pixel.
        :param point_cloud: array of shape (number of points, 3), Point cloud information, each point represented by  (x, y, z).


        :return: bounding_box: array of shape (2, 2) or array of shape (3, 2)
                 mask: array of shape (n_points, 1) or array of shape (width, height)
        '''


class Custom(Localization):
    def __init__(self, a1, a2, a3, ):
        raise NotImplementedError

    def display(self, **kwargs):
        raise NotImplementedError
