![rokin logo](rokin.png)

# Robot Forward Kinematic in Python 

---
# Dependencies

* [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [wzk](https://github.com/scleronomic/WerkZeugKasten)

# Installation
```commandline
conda install eigen
pip install git+https://github.com/scleronomic/rokin
```

# Example with existing robots

```python
from rokin.Robots import JustinArm07
# Other available robots are JustinFinger03 and Justin19
robot = JustinArm07()

print(robot.n_dof, robot.n_frames)
q = robot.sample_q(shape=100)
f = robot.get_frames(q)
print(f[0, -1, :, :])
```

# Load and use your own robot
* TODO: load urdf file and create the respective C++ files via the python setup scripts
* Link those new scripts to the main library

# Future Work
* prismatic joints
* coupled joints
* custom joints
* visualize the robot with pyvista