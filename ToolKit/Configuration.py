import yaml
import sys, os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

def readConfiguration(file_name):
    cfg_file = open(_root_path+'/Config/'+file_name+'.yaml')
    return yaml.load(cfg_file)
