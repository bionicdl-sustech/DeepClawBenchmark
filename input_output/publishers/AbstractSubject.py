import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)


class AbstractSubject(object):
    '''An abstract subject class.
    Provide abstract register, remove and notify methods, all publisher class should inherit this class.
    '''
    def registerObserver(self, observer):
        raise NotImplementedError(
            ' registerObserver method does not implement. ')

    def removeObserver(self, observer):
        raise NotImplementedError(
            ' removeObserver method does not implement. ')

    def notifyObserver(self):
        raise NotImplementedError(
            ' notifyObserver method does not implement. ')

    def sendData(self, data):
        raise NotImplementedError(
            " sendData method does not implement. ")
