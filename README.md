# RNM 
path planning launch
    
    clion anpassen wie besprochen
    catkin_make  (wird mit fehler abschließen, dann einfach nochmal catkin_make)
    roslaunch franka_example_controllers join_....._sim.launch
    rosrun forward_kin forward_kin_node
    dann den path planning node in clion launchen
    
## Needed packages
Eigen

    sudo apt install libeigen3-dev

Install libcc
    
    cd ~
    git clone https://github.com/danfis/libccd
    cd libccd
    cd src
    make
    sudo make install

FCL

    cd ~
    git clone https://github.com/flexible-collision-library/fcl
    cd fcl
    mkdir build
    cd build
    cmake ..
    make
    
## Dependencies

Libfranka

    sudo apt install ros-melodic-libfranka

## Startup

    roslaunch franka_example_controllers joint_position_example_controller_sim.launch
    rosrun franka_example_controllers test_move_node _command_topic:=/joint_position_example_controller_sim/joint_command


### Simulation

    roslaunch franka_example_controllers joint_position_example_controller_sim.launch
    rosrun franka_example_controllers test_move_node _command_topic:=/joint_position_example_controller_sim/joint_command

### Real Panda

    roslaunch franka_example_controllers joint_position_example_controller.launch
    rosrun franka_example_controllers test_move_node _command_topic:=/joint_position_example_controller/joint_command


## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
