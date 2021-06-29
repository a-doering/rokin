# import numpy as np


def print_justin_config(q):
    print('Torso:', q[:3])
    print('Right:', q[3:10])
    print('Left :', q[10:17])
    print('Head :', q[17:19])
