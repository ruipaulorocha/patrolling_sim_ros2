# `patrolling_sim_ros2`

## Description

This package contains the implementation of several algorithms for multi-robot patrolling and a general structure of a PatrolAgent that can be extended to implement other ones.

The [original code base](https://wiki.ros.org/patrolling_sim) was developed by [David Portugal](davidbsp@isr.uc.pt) at ISR - University of Coimbra during his PhD thesis work advised by [Prof. Rui P. Rocha](mailto:rprocha@isr.uc.pt). You can freely reuse the code in your own academic projects provided that you give credit to the authors in your related publications, by citing the following reference:

_D. Portugal & R. P. Rocha (2016). Cooperative Multi-Robot Patrol with Bayesian Learning, Autonomous Robots, 40(5):929-953, Springer. DOI: [10.1007/s10514-015-9503-7](https://doi.org/10.1007/s10514-015-9503-7)_


Besides porting the code base to ROS Jazzy distro, this version of the package extends previous versions of `patrolling_sim` with an improved structure of the code that allows easy integration of new algorithms, an improved navigation configuration that allows the robots to move at 1 m/s and to avoid most of conflicting situations, and a better management of the experiments and generation of the results.

## Dependencies

Add to your ROS workspace the packages below which need to be compiled from source. Follow the instructions available on each GitHub repository about these packages' dependencies.

- [`stage_ros2`](https://github.com/ruipaulorocha/stage_ros2)
- [p3dx\_ description\_ ros](https://github.com/ruipaulorocha/p3dx_description_ros.git)

Afterwards, install the other required dependencies with the following command, which assumes that you're in your ROS workspace's root directory:
```bash
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --ignore-src -y
```

## Usage

To run a simulation in Stage: `./stage_multi_robot_patrolling.py`

Several maps are available in the `maps` folder. For map X, the patrol graph is visible in the file
`maps/X/X-graph.png` 

Several algorithms have been implemented in the `src` folder. Each method is implemented through a class 'X_Agent' that inherits from the abstract class `PatrolAgent` many common services and functions.

Results of the experiments are stored in the `results` folder. After 30 minutes, the experiment terminates and the results will be available in the files
`results/{map}_{n.robots}/{algorithm}/{machine}/{date}*`.

The following result files are produced:

1) `info.csv` - contains a summary of the results of the experiment in a CSV format with the following values:

  Map ; N. robots ; Goal wait time ; Communication delay ; Navigation module ; MRP Algorithm ; MRP Algorithm parameters ; Machine ; Date ; 
  Sim Time ; Real time ; Interferences ; Termination status ; Idleness min ; avg	; stddev ; max ; 
  Interference rate ; Total visits ; Avg visit per node ;   Complete patrol cycles

2) `results.txt` - contains some information about the evolution of results in a text format

3) `idleness.csv` - contains the following results in a CSV format 

  Time ; Robot ; Node ; Node Idleness ; Interferences 

4) `timeresults.csv` - contains evolution over time of the following results in a CSV format

  Time ; Idleness min ; avg ; stddev ; max ; Interferences



The script can be extended to run multiple experiments in a single session, by just adding new commands like the one in the examples (possibly with different parameters).

