from .Robot import Robot, import_robot_cpp

# Academic 2D Robots
from .SingleSphere import SingleSphere, SingleSphere02, SingleSphere03
from .StaticArm import StaticArm, StaticTree
from .MovingArm import MovingArm, MovingArm03, Blob03, MovingTree
# Test Robots
from .DummyCoupled05 import DummyCoupled05
from .Dummy01 import Dummy01


# Agile Justin DLR
from .Justin19.Justin19 import Justin19, get_com
from .JustinArm07.JustinArm07 import JustinArm07
from .JustinFinger03.JustinFinger03 import JustinFinger03
from .JustinHand12.JustinHand12 import JustinHand12
from .JustinHand12Cal.JustinHand12Cal import JustinHand12Cal
from .JustinBase03.JustinBase03 import JustinBase03
from .JustinBase05 import JustinBase05

from .util import str2robot
