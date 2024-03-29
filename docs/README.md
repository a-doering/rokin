![rokin logo](rokin.png)

# Robot Forward Kinematic in Python 

---
# Dependencies

* [eigen](https://gitlab.com/libeigen/eigen)
* [sympy](https://github.com/sympy/sympy) 
* [wzk](https://github.com/scleronomic/WerkZeugKasten)

# Installation

1. install eigen 
   1. either use conda<br/>
      `conda install eigen`
      
   2. or specify the location of your version<br/>
      `export EIGEN_INCLUDE_DIR="your/path/to/eigen3"`

2. install rokin<br/>
`pip install git+https://github.com/scleronomic/rokin`


Note that no version is specified yet.
For upgrading / reinstalling the recommended procedure is to first uninstall rokin manually via<br/>
`pip uninstall rokin`

# Example with existing robots

```python
from rokin.Robots import JustinArm07
# Other available robots are JustinFinger03, JustinHand12, Justin19

robot = JustinArm07()  # The first time you call a robot, C++ code will 
                       # be generated. This may take a few seconds.
                       # Just run your scrip again. 
                       # Now everything should work

print(robot.n_dof)     # 
print(robot.n_frames)  # 
print(robot.limits)    # specify the lower and upper bound for each joint 
q = robot.sample_q(shape=100)  # Generates random configurations [shape x n_dof]
f = robot.get_frames(q)  # Calculates homogeneous matrices [n_frames x 4 x 4]
                         # for each configuration in q 
print(f[0, -1, :, :])
```
# Visualization
You still have to specify the directory of the meshes manually as 
they are not included in this git repository.
Ask the author if you need the files.
`export ROKIN_MESH_DIR="your/path/to/the/meshes"`
```python
# Visualize the robots
import numpy as np
from rokin.Robots import JustinFinger03
from rokin.Vis.robot_3d import justin_arm_07_interactive, robot_path_interactive

# play with all the sliders to get a feeling for the robot kinematic 
justin_arm_07_interactive(show_frames=[7],  # frames of the robots to plot
                          additional_frames=[np.eye(4)])  # fixed frames to plot 

# Pyvista blocks the execution as long as a window is open, so close the window to continue the code

robot = JustinFinger03()
q = robot.sample_q(50)
robot_path_interactive(q=q, robot=robot, 
                       mode='meshes',  # 'spheres' as alternative
                       show_frames=[5],
                       frames_scale=0.03)  # size of the frames [m]

# you can also directly generate a gif
robot_path_interactive(q=q, robot=robot, gif='robot.gif')
```
![PyVista Example for JustinArm07](pyvista_example.jpeg)

# Load and use your own robot
* TODO: load urdf file and create the respective C++ files via the python setup scripts
* Link those new scripts to the main library

# Future Work
* prismatic joints
* coupled joints
* custom joints
