# ROSI Simulator (sim_rosi)

This repository contains the **ROSI** robot simulator implemented in CoppeliaSim® in a ROS package format.

The model is fully operational, providing divers actuators and sensors integrated to the ROS framework. 
There is also an industrial yard simulation scene for implementation testing in representative scenario.

ROSI is a [Instituto Tecnológico Vale](http://www.itv.org/) project in partnership with [GSCAR](http://www.coep.ufrj.br/gscar/) group (COPPE/UFRJ).

The content here may be beneficial for robotics classes and also your research. 
Feel free to use/modify it for spreading Knowledge in your classes! :)

# Package description

This repository is structured as a ROS package. The folders organization is as follows:

- `config` - Contains the **.yaml** file with simulation parameters. You may change them accordingly to your needs.¹

- `coppeliaSim_content` - Contains ROSI model and scenes for CoppeliaSim® simulator.

- `launch` - Contains ROS launch files example.

- `msg` - Message files needed to interact with the Rosi simulated model.

- `script` - Example nodes in Python for interacting with the simulated model.

- `urdf` - Contains ROSI URDF model description.

- `vrep_content` - Contains the simulation scenes for the challenge. You may load them inside V-REP simulator.

Notes:
1-Parameters in `simulation_parameters.yaml` only take effect if you call the `load_parameters.launch` launch file prior pressing `play` in the simulation.

# Installation

This installation steps considers that you are running a **Ubuntu 18.04.5 LTS (Bionic Beaver)** OS with **ROS Melodic** 
`full-desktop` install and **CoppeliaSim® 4.1.0 (rev 1) Edu**. It might work in another software versions though. 
Also, we consider that you are current with Linux and ROS basics.

Links for installing [Ubuntu](https://releases.ubuntu.com/18.04/), 
[ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). We explaing how to configure CoppeliaSim® later in this text. 

Section notes:
1-No, never tried it on Windows machines. Good luck with that. :P

## Installation advices

By covention on this installation steps, all boxes starting with a `$` mark means that you should run the command on a Linux terminal. 

As an example:
``` 
$ sudo apt update
```
The above line means that you should run the command `sudo apt update` on your terminal. 
**Notice that** the `$` stands for a terminal command input and **do not** belongs to the command line syntax itself.

If you want to directly copy and paste this `README.md` commands on the terminal, remember that you **should not** copy the `$` mark, and the keyboard shortcut to paste on the terminal is `CTRL + SHIFT + V`.

The common issues for each step are directly addressed in a **Troubleshooting** remark on the end of the specific step instructions.

## Installation steps

Follow the steps below to configure your V-REP along with ROS:

**2.** Download **CoppeliaSim® 4.1.0 (rev 1) Edu** from the [Coppelia Robotics website](https://www.coppeliarobotics.com/downloads).

**3.** Unzip CoppeliaSim® (preferentially) to your **home** and rename the folder as `coppelia_sim`. 
From now on, we will call its folder path as `<coppelia_sim>`, e.g. `<coppelia_sim> == /home/your_user/coppelia_sim/`.

**1.** Clone and download this repository package to your **ROS Workspace src folder** (`../catkin_ws/src`) folder with the name `rosi_defy`:

```
$ git clone https://github.com/filRocha/sbai2019-rosiDefy rosi_defy
``` 






--------------------









**4.** Add both the CATKIN_WS and V-REP folder location to your `.bashrc`, an alias to run it, and source the `.bashrc` again: 
```
$ echo "export ROS_CATKIN_WS='<path_to_your_catkin_ws_folder>'" >> $HOME/.bashrc
$ echo "export VREP_ROOT='<path_to_your_vrep_folder>'" >> $HOME/.bashrc
$ echo "source $ROS_CATKIN_WS/devel/setup.bash" >> $HOME/.bashrc
$ echo "alias vrep=$VREP_ROOT/vrep.sh" >> ~/.bashrc
$ source $HOME/.bashrc
```
Remember to insert the path to your CATKIN_WS and V-REP folder in this command.

(All instructions consider that you use `bash`. If you use `.zsh`, you know what to do ;)


**5.** Test the V-REP functionality by running directly on your terminal the following command:
```
$ vrep
```
(Notice that you have created this command on the last step using the `alias` command on your `.bashrc`)


**6.** Clone recursively the V-REP/ROS interface and V-REP Velodyne2Ros plugin to your `catkin_ws/src`:
```
$ cd $ROS_CATKIN_WS/src/
$ git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git vrep_ros_interface
$ git clone https://github.com/filRocha/vrep_plugin_velodyne.git
```
(More information and credits about this interface can be found on its [Github repository](https://github.com/CoppeliaRobotics/v_repExtRosInterface.git)



**7.** Install some support packages:
```
$ sudo apt install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joint-state-publisher
```


**8.** We have just installed a new tool for compiling the catkin workspace: `catkin build`. If you use `catkin_make`, you have to clean your workspace and perform a fresh new compilation:
```
$ cd $ROS_CATKIN_WS
$ catkin clean
$ catkin build
$ source $HOME/.bashrc
```

Notice that you may not use `catkin_make` to compile your workspace anymore.

**Troubleshooting**: 
- The `catkin clean` command just deletes the `..catkin_ws/devel` and `..catkin_ws/build` folders from your ROS Workspace. If the `catkin clean` command returns an error (e.g. the workspace was not correctly indentified), you can just remove those folders **manually**.

**9.** Some messages from our `rosi_defy` package should be referenced in the `vrep_ros_interface` package. To do that:

9.1 Insert their namespace and names in `<vrep_ros_interface>/meta/messages.txt` file:
```
$ echo -e "rosi_defy/ManipulatorJoints\nrosi_defy/RosiMovement\nrosi_defy/RosiMovementArray\nrosi_defy/HokuyoReading" >> $ROS_CATKIN_WS/src/vrep_ros_interface/meta/messages.txt
```

9.2 Tell `vrep_ros_interface`that it depends on the `rosi_defy` package by adding 
```
<depend>rosi_defy</depend>
```
to `vrep_ros_interface/package.xml`.

9.3 Add `rosi_defy` package dependence on the `vrep_ros_interface/CMakeLists.txt`:
```
set(PKG_DEPS
  ... (many many other packages)
  rosi_defy
)
```

Compile again your `catkin_ws` using
```
$ catkin build
```

**10.** If your compilation runs well, there is now a ros interface and RosVelodyne libraries called `libv_repExtRosInterface.so` and `libv_repExtRosVelodyne.so`, respectively, on `<catkin_ws>/dev/lib/` folder. You must copy it to the V-REP folder:
```
$ cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosInterface.so $VREP_ROOT
$ cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosVelodyne.so $VREP_ROOT
```
(Notice that, for further events, every time you add new custom ROS messages to the interface, you have to re-compile this library and re-copy it to `$VREP_ROOT`.


**11.** Everything should be set up for now. To run the simulation, you should first (always!) run ROS:
```
$ roscore
$ vrep
```
Open the scene `<rosi_defy>/vrep_content/challenge_scenario.ttt` in V-REP and play it. You should be able to see the simulator topics being published with `rostopic list`. 

Additionally, if you have a joystick, you can run the `rosi_joy.py` example node to see how the communication with the robot works.

**NOTICE** that you have to run the `roscore` **ALWAYS** before `vrep` in order to work. If you stop ROS master, you have to close V-REP and run it all again.

# Hello World!

To first run your simulator, do the following:

**1.** Open a new terminal and run `$ roscore` to enable `ROSMASTER`. You **always** have to run the ROS master before CoppeliaSim® (at least if you want things to work properly.)

**2.** In another terminal window, run `$ coppelia_sim` (this should work if you have created successfully CoppeliaSim® alias on `.bashrc`.) 
In the negative case, run it directly using `$ <coppelia_sim_folder>/coppeliaSim.sh`.

- TODO adequar o nome do arquivo a ser aberto
**3.** Go to `File > Open Scene...` and locate the CoppeliaSim® scenario in `<sim_rosi>/coppeliaSim_content/challenge_scenario.ttt`. 
You have also available the ROSI model that can be directly loaded in another scenario. It can be also found in the CoppeliaSim® model browser (simulator's left side column).

**4.** Start the simulation by going to `Simulation > Start simulation` (or just press the `play` button.) At this point, you should be able to see all ROSI topics appearing in the ROS framework.

**5.** You may find ROS node examples in `<sim_rosi>/script/`. 


# Simulation Parameters

The simulation may run slow due to its complexity and amount of running modules in parallel. 
We encourage turning off functionalities that you do not need in your experiments.
For that, you can tweak some parameters in the `<sim_rosi>/config/simulation_parameters.yaml` file.
Parameters are loaded when you call the `<sim_rosi>/launch/load_parameters.launch` prior running the simulation.

The following flags are available:
- TODO confirmar os parametros de simulacao
- `simulation_rendering` - `Boolean` - Controls the rendering of V-REP visualization. Disable it for better performance.

- `velodyne_processing` - `Boolean` - If your code do not rely on Velodyne data, you can disable this sensor here and speed up the simulation.

- `kinect_processing` - `Boolean` - If your code do not rely on Kinect data, you can disable this sensor here and speed up the simulation.

- `hokuyo_processing` - `Boolean` - If your code do not rely on Hokuyo data, you can disable this sensor here and speed up the simulation.

- `hokuyo_lines` - `Boolean` - Turn this ON if you want to see hokuyo detection lines. 

# ROSI2ROS

This section shows how to interact with the simulated ROSI in ROS framework. It shows the published/subscribed topics, along with its message types and brief comments.

The standard is:
- `/topic_namespace/topic_name` - `<topic_message/Type>` - A brief description of its content.

As good systematic boys, all variables are expressed in the International System of Units (SI).

## ROSI model publishes to :

- `/manipulator/joints_pos_sensor` - `<sim_rosi/ManipulatorJoints>` - The embedded manipulator joints position. 

- `/manipulator/wrist_ft` - `<geometry_msgs/TwistStamped>` - The embedded manipulator Force/Torque sensor output. It gives two vectors of linear and angular forces and torques, respectively. Axis order is **x**, **y**, **z**.

- `/rosi/arms_joints_position` - `<sim_rosi/RosiMovementArray>` - Rosi tracked arms position in \[radians\].

- `/rosi/kinect_joint` - `<std_msgs/Float32>` - Kinect joint position in \[radians\].

- `/sensor/gps` - `<sensor_msgs/NavSatFix>` - Emulated GPS sensor output.

- `/sensor/hokuyo` - `<sim_rosi/HokuyoReading>` - Emulated hokuyo output. It gives a vector of 3D coordinates of detected point with respect to hokuyo.

- `/sensor/imu` - `<sensor_msgs/Imu>` - Emulated IMU sensor output.

- `/sensor/kinect_depth` - `<sensor_msgs/Image>` - Emulated kinect depth image output.

- `/sensor/kinect_info` - `<sensor_msgs/CameraInfo>` - Emulated kinect information.
  
- `/sensor/kinect_rgb` - `<sensor_msgs/Image>` - Emulated kinect rgb image output.

- `/sensor/manipulator_tool_cam` - `<sensor_msgs/Image` - Emulated camera on the manipulator tool.

- `/simulation/time` - `<std_msgs/Float32>` - V-REP simulation time in \[seconds\]


- `/velodyne/points2` - `<sensor_msgs/PointCloud2>` - Emulated Velodyne output.


## ROSI subscribes to:
- TODO reescrever os limites de publicacao
- `/manipulator/joints_target_command` - `<sim_rosi/ManipulatorJoints>` - Sets the UR-5 joints desired angular position. Each joint has a built-in PID controller. One may find more UR-5 info in [here](https://www.universal-robots.com/media/50588/ur5_en.pdf).

- `/rosi/command_arms_speed` - `<sim_rosi/RosiMovementArray>` - Sets the tracked arms angular velocity in \[radians/s\]. Command limits in \[-0.52, 0.52\] rad/s

- `/rosi/command_kinect_joint` - `<std_msgs/Float32>`- Sets the kinect joint angular position set-point. Joint limits are \[-45°,45° \]. It has a built-in PID controller with maximum joint speed of |0.35| rad/s.

- `/rosi/command_traction_speed` - `<sim_rosi/RosiMovementArray>` - Sets the traction system joint angular velocity in \[radians/s\]. The traction system drives simultaneously both the wheel and the tracks. Command limits in \[-37.76, 37.76\] rad/s.

PS. Topics can be renamed by changing model parameters in Coppeliasim® scene hierachy.

## Thanks

- We would like to thank **Marc Freese** and the Coppelia Robotics team for releasing and maintaing the amazing CoppeliaSim® simulator.

- These noble gentlemen greatly contributed to this simulator: Amauri Coelho Ferraz, Raphael Pereira Figueiredo da Silva, Evelyn Soares Barbosa and Wagner Ferreira Andrade.

- Kinova Gen3 manipulator xacro model obtained in [ros_kortex](https://github.com/Kinovarobotics/ros_kortex).

## ROSI Challenge

The present simulator is based on the **ROSI Challenge** competition repository. 
Participating teams had to perform autonomous tasks with ROSI in a representative mining port scenario.

The following are the ROSI Challenge finalists' codes (you may be inspired by them):

- **1st - Time IFES Vitória**: https://github.com/AntMol8/Time_Ifes_Vitoria
- **2nd - Pra Valê**: https://github.com/vinicius-r-silva/Pra_Vale
- **3rd - AAI Robotics**: https://github.com/ara1557/AAI_robotics
- **4th - Hofs**: https://github.com/Gustavo-Hofs/hofs_rosi_challenge_2019
- **5th - PPGEAS - UFSC**: https://github.com/feressalem/rosi_challenge_ppgeas_ufsc
- **6th - ForROS**: https://github.com/raphaellmsousa/ForROS
- **7th - Pyneapple**: https://github.com/DiegoFr75/pyneapple
- **8th - Taura Bots**: https://github.com/alikolling/taura_Rosi_challenge
- **9th - Titãs da Robótica**: https://github.com/jeanpandolfi/TitasdaRoboticaRosiChallenge

We strongly recommend the teams' to maintain their repositories online and public. However, we are not responsible for their content and situation.

Related content: [ROSI Challenge official website](https://www.sbai2019.com.br/rosi-challenge), [original Github repository](https://github.com/filRocha/rosiChallenge-sbai2019), and [news report](http://www.itv.org/noticia/vale-patrocina-desafio-no-14o-simposio-brasileiro-de-automacao-inteligente-sbai/) from the Intstituto Tecnológico Vale website.


# I have found something wrong!

Great! Please, open a new **issue** directly on this repository, or send your **pull request**.

If you have any suggestions, comments or compliments, you can reach the official challenge contact: `filipe.rocha@coppe.ufr.br`

Doubts about the simulator MUST be treated as an **issue**! Thanks.

# Hey...
Have fun!

