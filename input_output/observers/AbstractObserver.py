import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)


class AbstractObserver(object):
    '''An abstract observer class.
    Provide abstract updata, display methods, all observer class should inherit this class.
    '''
    def update(self, data):
        raise NotImplementedError(
            ' update method does not implement. ')

    def display(self):
        raise NotImplementedError(
            ' display method does not implement. ')
