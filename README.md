# FLYBO Fork of NBVP: Receding Horizon Next Best View Planning

This is a fork of the public AEP package that we refactor in order to fit within the FLYBO system.
This is in particular in order to factorize the common routines shared by autonomous exploration planners to allow for fair, accurate comparative evaluation of planning systems. More info on the project's website www.flybo.org.

## Citation

```
@inproceedings{bircher2016receding,
  title={Receding horizon "next-best-view" planner for 3D exploration},
  author={Bircher, Andreas and Kamel, Mina and Alexis, Kostas and Oleynikova, Helen and Siegwart, Roland},
  booktitle={2016 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1462--1468},
  year={2016},
  organization={IEEE}
}
```

```
@inproceedings{Brunel3DV2021,
  TITLE = {FLYBO: A Unified Benchmark Environment for Autonomous Flying Robots},
  AUTHOR = {Brunel, Anthony and Bourki, Amine and Strauss, Olivier and Demonceaux, C{\'e}dric},
  BOOKTITLE = {9th International Conference on 3D Vision},
  ADDRESS = {Online, United Kingdom},
  YEAR = {2021}
}
```

# FLYBO Installation

Installation instructions are provided in the [FLYBO wiki](https://github.com/anthonybrunel/FLYBO/wiki). Feel free to create an issue if you have any questions or problems related to this package.

# Original ReadMe

# Receding Horizon Next Best View Planning

The next best view planner is a real-time capable exploration path planner. From the current pose it expands a tree to find a next pose that gives a high exploration gain. This gain reflects the exploration of space that is not yet (sufficiently) known. As the vehicle proceeds on the path, the tree is recomputed, taking into account the new information from the sensor.

This README gives a short overview. For more information refer to the [wiki](https://github.com/ethz-asl/nbvplanner/wiki).

# Planner installation and execution

To run the current version, compile the package nbvplanner. To get it navigate to the source folder of your ros workspace:

```sh
git clone https://github.com/ethz-asl/nbvplanner.git
cd nbvplanner
git submodule init --
git submodule sync --recursive
git submodule update --recursive
cd ..
```

Moreover, make sure you have all the necessary libraries:
```sh
apt-get install ros-<distro>-octomap-*
apt-get install python-catkin-tools
catkin build
```

For a simulation demo launch

```sh
roslaunch interface_nbvp_rotors flat_exploration.launch
```

Tested under ROS Indigo and Jade.

Further instructions for the visualization of the exploration progress, as well as more demo scenarios and descriptions of the parameters can be found in the [wiki](https://github.com/ethz-asl/nbvplanner/wiki).


If you use this software in a scientific publication, please cite the following paper:
```
@inproceedings{bircher2016receding,
  title={Receding horizon "next-best-view" planner for 3D exploration},
  author={Bircher, Andreas and Kamel, Mina and Alexis, Kostas and Oleynikova, Helen and Siegwart, Roland},
  booktitle={2016 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1462--1468},
  year={2016},
  organization={IEEE}
}
```

# Credits

This algorithm was developed by [Andreas Bircher](mailto:bircher@gmx.ch) with the help and support of the members of the [Autonomous Systems Lab](http://www.asl.ethz.ch). The work was supported by the European Commission-funded project [AEROWORKS](http://www.aeroworks2020.eu/).

# Contact

You can contact us for any question or remark:
* [Andreas Bircher](mailto:bircher@gmx.ch)
* [Kostas Alexis](mailto:konstantinos.alexis@mavt.ethz.ch)
