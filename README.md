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

Also, this symbol `<>` states for absolute path. 
You have to replace it for the required component path. 
For instance, if we write `<catkin_ws>` referring to the ROS workspace folder, you replace this stretch by `/home/your_user/wherever_you_put_your_workspace/catkin_ws/`. 
Easy, right?

The common issues for each step are directly addressed in a **Troubleshooting** remark on the end of the specific step instructions.

## Installation steps

Follow the steps below to configure your ROSI simulator to operate along with ROS:


**7.** Install some ROS support packages and required Python 3.8:
```
$ sudo add-apt-repository ppa:deadsnakes/ppa
$ sudo apt install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joint-state-publisher xsltproc python3.8 python3-pip
$ sudo python3 -m pip install xmlschema
```




**1.** With the `python-catkin-tools` package, we have just installed a new tool for compiling ROS packages: `catkin build`¹.
If you use `catkin_make` tool, we first have to delete current compiled files within `<catkin_ws>` folder, and then recompile
it using the new tool.

```
$ cd <catkin_ws>
$ catkin clean
% caktin build
```

**Note**: We need `catkin build` for properly building the CoppeliaSim® ROS Interface.





**2.** Download **CoppeliaSim® 4.1.0 (rev 1) Edu** from the [Coppelia Robotics website](https://www.coppeliarobotics.com/downloads).




**3.** Unzip CoppeliaSim®, put it in a convenient folder, and rename it as `coppelia_sim`, for simplicity.




**1.** Clone and download this repository package to `<catkin_ws>/src/` folder with the name `sim_rosi`:

```
$ git clone https://github.com/filRocha/sim_rosi
``` 




**4.** Add the following lines to your `.bashrc`. For opening the `.bashrc` file, run `$ nano ~/.bashrc`

```
export ROS_CATKIN_WS='<catkin_ws>'
export COPPELIASIM_ROOT_DIR='<coppelia_sim>'
source <catkin_ws>/devel/setup.bash
alias coppelia_sim=$COPPELIASIM_ROOT_DIR/coppeliaSim.sh
```

An then, reload your terminal environment with `$ source ~/.bashrc`.

**Note 1**: Remember to replace the path to your CATKIN_WS and V-REP folder in this command.

**Note 2**: All instructions consider that you use `bash`. If you use `.zsh`, you probably know what to do ;)




**5.** To test CoppeliaSim®, open a new terminal and run `$ coppelia_sim`. 
You may close it if everything went alright.

**Note**: You have created this command on the last step using the `alias` command on your `.bashrc`. ;)

**Troubleshooting 1**: If you receive an error like `bash: <some_folder>/ccoppeliaSim.sh: No such file or directory`,
check if the path is correctly set in your `.bashrc`. 

**Troubleshooting 2**: When starting, CoppeliaSim® prints its initialization steps on terminal. 
If it crashes while starting, try finding some clues there.




**5.** Update your CoppeliaSim® `libPlugin` folder
```
$ cd <coppelia_sim>/programming/
$ rm -rf ./libPlugin
% git clone https://github.com/CoppeliaRobotics/libPlugin
```




**6.** Clone recursively the CoppeliaSim®/ROS interface and CoppeliaSim Velodyne2Ros plugin to `<catkin_ws>/src/`:
```
$ cd <catkin_ws>/src
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
$ git clone https://github.com/ITVRoC/coppeliasim_plugin_velodyne.git
```

and checkout the CoppeliaSim®/ROS interface to `Melodic` branch
```
$ cd <catkin_ws>/src/sim_ros_interface/
$ git checkout melodic
```

Now compile the workspace with `$ catkin build`.

**Note**: take some time to browse those repositories and credit their authors. :)

**Troubleshooting**: If you have a `CMake` version problem, there is a special session describing how to fix it by the
end of this installation steps.




**9.** Now we reference `sim_rosi` custom messages in the `sim_ros_interface` so the CoppeliaSim® can recognize those messages
while communicating with ROS. To do so:

9.1 Insert their namespace and names in `<sim_ros_interface>/meta/messages.txt` file:
```
$ echo -e "sim_rosi/HokuyoReading\nsim_rosi/ManipulatorJoints\nsim_rosi/RosiMovement\nsim_rosi/RosiMovementArray" >> $ROS_CATKIN_WS/src/sim_ros_interface/meta/messages.txt
```
 
Now, inform to the `sim_ros_interface` package that it depends on the `sim_rosi` package.

9.2 Open the file `<sim_ros_interface>/package.xml` file and add 
```
<depend>sim_rosi</depend>
```
after the last `<depend> xxx </depend>` statement.


9.3 Open `<sim_ros_interface>/CMakeLists.txt` and add to the proper place:

```
set(PKG_DEPS
  ... (many many other packages)
  sim_rosi
)
```

9.4 Compile again your ROS workspace `<catkin_ws>`
```
$ catkin build
```




**10.** If everything went ok, copy generated libraries¹ to your CoppeliaSim folder:

```
$ cp $ROS_CATKIN_WS/devel/lib/libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
$ cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR
```

**Note**: This specific step should be done everytime you add new custom messages to the CoppeliaSim® ROS interface.






**11.** Everything should be set up now. To run the simulation, you have to first start ROSCORE and later open the simulator:
```
$ roscore
$ coppelia_sim
```
Open the scene `<rosi_defy>/vrep_content/challenge_scenario.ttt` in V-REP and play it. You should be able to see the simulator topics being published with `rostopic list`. 

Additionally, if you have a joystick, you can run the `rosi_joy.py` example node to see how the communication with the robot works.

**NOTICE** that you have to run the `roscore` **ALWAYS** before `vrep` in order to work. If you stop ROS master, you have to close V-REP and run it all again.






PAREI AQUI PAREI AQUI
--------------------







### Fixing CMake Version

If you have CMake version problem when trying to compile the CoppeliaSim®/ROS Interface `sim_ros_interface`, try the following:

- Check your CMake version with `$ cmake --version`. You'll need version `>=3.16`.

- Go to [this link](https://cmake.org/download/) and download the latest version (or any version `>=3.16`) `.sh` file. 
For instance, considering version `3.19.3`, the file name is `cmake-3.19.3-Linux-x86_64.sh`. 
  We'll use this file name from now on.

- Move this binary file to `/opt/`. Run `$ sudo mv ./cmake-3.19.3-Linux-x86_64.sh /opt/`.

- Navigate to `/opt`: `$ cd /opt/`.

- Make the binary executable with `$ sudo chmod +x ./cmake-3.19.3-Linux-x86_64.sh`.

- Run it with `$ sudo bash ./cmake-3.19.3-Linux-x86_64.sh*`. You may have to press `Yes` twice.

- Create symbolic links for CMake with `$ sudo ln -s /opt/cmake-3.19.3-Linux-x86_64/bin/* /usr/local/bin`.

- Test the installation by running again `$ cmake --version`.

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

