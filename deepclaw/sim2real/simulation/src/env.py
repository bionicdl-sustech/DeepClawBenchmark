from os.path import dirname, join, abspath
from pyrep import PyRep

class Env(PyRep):
    '''
    Open v-rep and the Scene you give 
    '''
    def __init__(self, file_name):
        '''
        para
        ------
            file_name : 'xxx.ttt' file
        '''
        super().__init__()
        self.launch(file_name)
    