import numpy as np

from rokin.Robots import JustinHand12, import_robot_cpp


class JustinHand12Cal(JustinHand12):
    def __init__(self):
        super().__init__()
        self.id = 'JustinHand12Cal'
        #  self.dh += np.array([[[-0.00641108,  0.00011819,  0.00065671, -0.00598573],
        #  [ 0.00072967, -0.00693897, -0.00716587, -0.00782515],
        #  [ 0.00070378, -0.00642417,  0.00538544, -0.00188082],
        #  [ 0.00069073, -0.00294551,  0.00533029, -0.00019698]],
        # [[ 0.00248494,  0.00261595, -0.00018759,  0.00020551],
        #  [-0.00026498,  0.0008781 , -0.00650776, -0.0051252 ],
        #  [-0.00026929,  0.00089211,  0.00560164,  0.00280679],
        #  [-0.00026015,  0.0009593 , -0.00128087,  0.00114982]],
        # [[-0.00166879, -0.00246209, -0.00257603,  0.00559203],
        #  [-0.00099885,  0.00268696, -0.00321219,  0.00457903],
        #  [-0.00102647,  0.0007944 ,  0.00609606,  0.00131402],
        #  [-0.00102808,  0.00046709, -0.00054434,  0.00021366]],
        # [[ 0.00196291,  0.00318276,  0.0013926 , -0.00120645],
        #  [ 0.00036432, -0.00142647,  0.00085215,  0.00121911],
        #  [ 0.000265  , -0.00334764, -0.0002128 ,  0.00379326],
        #  [ 0.00032718, -0.00157902,  0.0005203 ,  0.00155414]]]).reshape(16,4)

        # CALIBRATED
        self.f_static[1::2] = np.array([[[-0.14041388,  0.6964277 ,  0.70375592, -0.008753  ],
                                         [-0.16868412, -0.71723294,  0.67610841, -0.04568244],
                                         [ 0.97561755, -0.02377744,  0.21818576,  0.12431245],
                                         [ 0.        ,  0.        ,  0.        ,  1.        ]],
                                        [[-0.18193017,  0.02798179,  0.98291324, -0.05103251],
                                         [-0.19734628, -0.98029594, -0.0086201 ,  0.0029209 ],
                                         [ 0.96330465, -0.19554253,  0.18386751,  0.1292965 ],
                                         [ 0.        ,  0.        ,  0.        ,  1.        ]],
                                        [[-0.21002886,  0.05765612,  0.97599367, -0.04598232],
                                         [-0.09710584, -0.99455386,  0.03785587,  0.02769158],
                                         [ 0.9728609 , -0.08682386,  0.21448376,  0.13884357],
                                         [ 0.        ,  0.        ,  0.        ,  1.        ]],
                                        [[ 0.43035205,  0.12216061, -0.8943567 ,  0.05180671],
                                         [-0.33738921,  0.9407565 , -0.03384864,  0.03071566],
                                         [ 0.8372369 ,  0.31631313,  0.44607216,  0.10283151],
                                         [ 0.        ,  0.        ,  0.        ,  1.        ]]])

        self._cpp = import_robot_cpp(robot=self)
