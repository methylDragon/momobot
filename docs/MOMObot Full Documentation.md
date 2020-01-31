# MOdular MObile (MOMObot) Robot Documentation
> Version 1.1
## Credits
This documentation has been put together with the combined efforts of members of the **SUTD Organisation of Autonomous Robotics**

### 2018 - 2019

- [methylDragon](https://github.com/methylDragon)
- [Shine16](https://github.com/shine16)
- [Fasermaler](https://github.com/fasermaler)
- [imossim](https://github.com/imossim)
- Bryan Kong
- Low En
- [Senrli](https://github.com/senrli)

### 2019 - 2020

- [Photon](https://github.com/1487quantum)
- [robobdo](https://github.com/robobdo)
- Jia Hwee
- Jerremy
- [darthnoward](https://github.com/darthnoward)

## Introduction

MOMObot is a service AGV built for extensibility and to roam autonomously using ROS!

## Hardware

### Dimensions

- 52x52x80 cm (just frame)
- 52x52x84 cm (including wheels)

Dimensions decided based on the requirements that MOMO has to fit inside lifts, and pass through doors. 

The ratio of 52 cm : 80 cm of the frame is to improve the aesthetics of the MOMO, by observing the golden ratio.

### Specifications
Side panels are tapered on one end to allow for a 270 degree field-of-view to ensure unobstructed LIDAR operation. Components on the bottom are shifted to the rear in addition to tapered side panels to allow for unobstructed view for the LIDAR.

MOMO uses rear wheel drive to ensure maximum stability when driving forward. Front wheel drive systems are inherently unstable due to fluttering of the castor wheel - undesirable oscillations during autonomous navigation (this was shown empirically as the original design was a front wheel drive).

This allows for the robot to deal with any sudden loss of front traction as well as traverse ramps.

**Design Payload Weight:** 20 kg
In actuality, MOMO can take the weight of 1 person (60+ kg) when both motors are moving. (However, it would be unable to perform pivot turns.)

#### Some quirks in selection and design
2040 Extrusions are utilised to allow for greater loading of frame by increasing the amount of material vertically. This increases the maximum strain which the frame can withstand.

4 corner pillars of MOMO comprise of L-shaped 4040s to prevent horizontal warping of the frame.

Reason for more extrusions in front:
- Expose more of the structure to allow lidar to function
- Would have to pay attention to how frame is supported 

Reason to do rear wheel drive
- Enable robot to handle uneven ground
    - Middle wheel drive - on uneven ground, castors lift
    - Front wheel drive - spirals out of control, oscillates during navigation

Motors are mounted with 2 thick aluminum plates
Motors that slide in from the side allow for ease of maintenance

Bottom layer braced with L shaped extrusions to stop battery from moving too much, as well as to provide a sturdier frame.

Momo as 4 seperate layers:
1. Motor and Battery
1.5. Lidar
2. Electronics - driver, relays, mcu, esc, all else
2.5. Screen
3. Computer
3.5. E-stop
4. Any thing else to be added

Castors chosen so as not to leave marks (stay away from rubber wheels -  first waiterbot used rubber castor wheels which left black marks at Fablab when navigating )

After a long day of running Momobot, there is a need to check gussets as they may loosen. Very important to check the bottom as there are over 20 gussets at the bottom securing the batteries in place.

Can be tipped without problem as frame is sturdy

Good design: 
Low CG - able to stabilize quicky after tilting

CG weight
Rear wheel drive - all weight on rear wheel, when accelerate does wheelie, so weight was shifted forward

### Gotchas, Hacky Stuff and Things to Take Note Of

Take note:
- Screws to motor -able to come out with vibrations
    - Remember to check gusset bolt tightness if MOMO has been operated with many vibrations 
- Caster wheels - have to use washers to compansate
- Rounded Motor mount bolts:
    - Motor mount mounted to extrusion with 6 bolts - some of the bolts are rounded and cannot be removed
- 2nd layer - to screen - the screen mount was not designed for Momo
- Arm not compatible with back plate, backplate not compatible with screen
- Bought mount, Could not extend to extrusions
- Arm to mount screen forced upwards to fit the screen inside. This however, locks the screen in place.
- Castor wheel mountings - Castor wheel mountings too big for screw.
- Screw to giant washer to washer to attach the caster wheels at the bottom. 
### To Dos
1. Redo Lidar Mount - side holder tolerance w
2. Implement a guard for the front LiDAR
3. Implement Rear LiDAR
4. Implement system to improve ease of removing back panel (i.e. magnets, hooks), current back panel is screwed on by 6x rhombus nuts.
5. Implement a charging port for both 55Ah Batteries and 7Ah Batteries


## Electronics


Powers the controlling system, including the lidar, VESC, Teensy, laptop , router as well as two solid state relays
Two solid state relays were used

### Electronic BOM

#### Power Source
- 2x 12V 55Ah Pb acid batteries (Connected in Series)
- 2x 12v 7Ah Pb acid batteries (Connected in Series)
- Terminal Blocks for Power Distribution
- 3x el cheapo (Taobao) variable voltage Buck (literally worth the buck) Converter (replace pls) (current settings 2x 19V, 1x 12V)

#### Sensors
- 1x LMS111 LiDAR
        - 20m range, 270 degree FOV
        - For obstacle detection, and mapping, for navigation
- Motor Encoders (Came With Motors)
    - Measure how far motors have turned, important for odometry
- GY-85 IMU
    - Important for Odomentry, provides another source
- Marvelmind Indoor GPS
    - Currently not in use, for a 3rd possible global pose data source

#### Actuators and Outputs
- 1x Waveshare 13.3" HDMI LCD (H) with case
        - https://www.waveshare.com/13.3inch-hdmi-lcd-h-with-case.htm
- 2x Flipsky 50A Continuous VESC v4.12 (Using JST-PH)
        - Normal ESC did not work for our specced motor
- 350W 24V Brushless DC Scooter Hub Motor
        - https://www.aliexpress.com/item/24V-36V-48V-8Inch-Electric-Wheel-Hub-Motor-350W-Brushless-Non-Gear-Hub-Motor-For-Electric/32837818637.html
- Teensy 3.2 (Original from PJRC)
- Cheap speakers


#### Safety and Power Control
-    1x Schnieder DC Circuit Breaker 125V 20A (2-way)
-    1x Schnieder DC Circuit Breaker 125V 16A (1-way)
-    2x CDG1-1DD/40A Solid State Relay
-    1x CDG1-1DD/25A Solid State Relay
-    Heat sinks
-    1x E-stop
-    1x electronics On/Off Switch

#### Connectors
-    8x XT90 Connectors (Male and Female)
-    2x 6mm Barrel Jacks
-    Laptop power adapter


### Start-Up, Shut-Down Procedure

#### Start-Up (FULL)
1. Check the battery leads and ensure that the batteries are connected in series 
2. Electronics breakers to be Switched to "ON"
3. Set the Green electronics switch to "ON"
4. Motor Breakers to be switched to "ON"
5. Set the E-stop to "OFF"

#### Shut-down (FULL)
1. Set the E-stop to "ON"
2. Switch the Motor breakers to "OFF"
3. Set the electronics switch to "OFF"
4. Set the electronics breaker to "OFF"
5. Disconnect battery leads

#### Start-Up (truncated)
This assumes the batteries have been connected beforehand
1. Electronics switch to be set to "ON"
2. E-stop set to "OFF"

#### Shut-down (truncated)
1. E-stop set to "ON"
2. Electronics switch to be set to "OFF"
3. Disconnect the battery leads


### Gotchas, Hacky Stuff and Things to Take Note Of
- 2x16AWG wires used to take high current out of battery, as we did not have thick enough wires at the time. The wire usage was not consistent, as some were salvaged PVC wires from the previous bot. Suggested to use all silicone coated wires with low gauge for higher termperature endurance and lower resistance.

- Encoder and PWM input wires from Teensy to the VESC was connected using jumper cables rather than specific JST-PH connectors. Encoder wires were spliced to 2, one to VESC, one to teensy, causing a mess of wires and potential intermittent connections.

- All bulk converter displayers are broken - they show a wrong voltage.

- VESC can be better positioned to be easier USB tunned, and the layout should be revised for easier switch fliping
  
- No voltage monitoring circuit included in either of the two electrical system, making monitoring and preparing for recharge difficult. 19v for fully charged, 18.1v for need to charge. Circuit needed to be implemented to prevent either of the batteries from over-discharging.

- The lack of a charging circuit made the life of the maintenance team difficult. Much more troubles of disconnecting the batteries for recharging and connecting back for operation

### Circuit Diagrams
* The robot consists of two power systems
* 24V with smaller battery capacity and 24V with bigger battery capacity
* Both 2 cells in series to boost the voltage for the motors, as well as the lidar

![](https://i.imgur.com/2eP14wi.png)
![](https://i.imgur.com/bqXtxwv.png)
![](https://i.imgur.com/biKbPmZ.png)
![](https://i.imgur.com/jlg5EWP.png)

### Motor Tuning
**VESC tuning**

Follow link for VESC tuning documentation
https://github.com/Fasermaler/Misc-Notes/blob/master/Flipsky%20VESC%204.12%20Documentation.md

(Or check the `Flipsky VESC 4.12 Documentation.md`)

**FOC signal**

VESC also has internal PID control which is not modified in the original MOMObot because the PPM signals sent to the motors have been PID-ed in the ROS stack.

- When tuning the PPM signal centre, max and min, ensure that the ROS stack is running.
- The neutral signal will be the "centre", max forward throttle will be "max" PPM and max reverse throttle "min" PPM. 
- The deadband of 10% is selected to allow MOMO to gain enough throttle to overcome initial traction.
- A PPM deadband of < 2% is not recommended as it means any small fluctuation in PPM will command motor response.

3 days work tuning the settings

Read lots of guides for Duty settings - values that work
After tuning the VESC's PPM and Duty Cycle settings, remember to write the settings else they will not be saved.



Momo charges alot
possible due to the I component of PID increasing when attempting to Pivot, then when transitioning to a forward movement, the built up I causes a surge in motor response.

Room for impovement
-FOC profile can be improved (FOC settings , will have lots of variables to tune to fit the curve better)
-positive, negative ramping time

### PID Tuning from the MOMObot side

**Edit the config file**
1. `roscd momobot/teensy/firmware/lib/config/` 
2. `nano momo_base_config.h`
3. Change these parameters
    - Differential drive
    - USE_ESC
    - Kp, Ki, Kd
    - Encoder pins, can be changed in hardware or code

Use the [Linorobot PID tuning guide](https://github.com/linorobot/linorobot/wiki/2.-Base-Controller
)!

## Software

Using Linorobot stack as it simplifies the tuning 

Linorobot to Momobot stack changes:
1. changed Linorobot to Momobot names in code
2. changed motor driver code

### Pre-Requisites
- ROS Proficiency
- Intermediate Linux/Ubuntu Command Line Proficiency
- Linorobot experience
    - https://linorobot.org

### Setting Up

#### Logging into MOMOBOT
1. Login to momobot, using the SUTD_LAB WiFi
 `ssh <USERNAME>@10.21.132.80` 
 or 
 `ssh momobot`

    - Note, this only works if you've configured an SSH Alias for momobot!
    - Put inside `~/.ssh/config`
    - Make the file if it doesn't exist using `sudo touch ~/.ssh/config` or `sudo nano ~/.ssh/config`
    
        Contents:

      ```shell
      Host momobot
      	# SUTD_LAB
          Port 22
          User <USERNAME>
          HostName 10.21.132.80
      ```

2. Access the teensy config file
    - `roscd momobot/teensy/firmware/lib/config`
    - Opening this file will expose the PID values for tuning


#### Setting Static IP for New Robots
A static IP is needed so as to be able to ssh into the robot 

1. Setup static IP

   ```bash
   $ route -n # Use this to check default gateway and netmask
   ```

   Netmask is the Genmask

   Gateway is Gateway

   Also check if you need a DNS setup. (SUTD requires this one: 192.168.2.100)

2. Then enable ssh

   ```bash
   $ sudo apt-get install openssh-server
   ```
To set Static IP, under WiFi connection settings, click edit connections.
Set IPV4, set manual, set static address, Netmask and Gateway, according to `route -n` in the commandline 

To work with SUTD wifi for internet, use DNS Server: `192.168.2.100`

#### Install ROS and Other Packages if needed

1. Run the install scripts from setup_scripts
    - Credits: https://github.com/methyldragon/quick-install-scripts
2. Specific order:
    - `./convenience_tools_install`
    - `./ros_lino_base_install`
    - `./robot_playground_install`
    - This should install your net tools, ROS, as well as the pre-requisite Linorobot stack.

3. Then, copy paste the scripts in the `src` directory inside a ROS workspace, preferably called `momobot_ws`

#### Setup the ~/.bashrc on MOMOBot as well as your Computer
> Otherwise you won't be able to get any ROS data!!!


### **On MOMObot**
1. `sudo nano ~/.bashrc`
2. Append this
    - PLEASE REMEMBER TO CHANGE THE THINGS IN <>
      Remember to save also!
        ```shell
        source /opt/ros/kinetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
      
        source /home/<USERNAME>/<ROS_WORKSPACE_NAME>/devel/setup.bash
      
        source ~/<ROS_WORKSPACE_NAME>/devel/setup.bash
        export LINOLIDAR=lms111
        export LINOBASE=2wd
      
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<USERNAME>/<ROS_WORKSPACE_NAME>/src
      
        alias costmap_reset="rosservice call /move_base/clear_costmaps"
      
        export ROS_IP=<IP ADDRESS OF ROBOT>
        export ROS_HOSTNAME=<IP ADDRESS OF ROBOT>
        ```
    ```
    
    ```

### **On Computer**

1. `sudo nano ~/.bashrc`
2. Append this
    - PLEASE REMEMBER TO CHANGE THE THINGS IN <>
        ```shell
        ip=$(ip addr show wlo1 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
        
        export ROS_MASTER_URI=http://<ROBOT_IP_ADDRESS>:11311
        export ROS_IP=$ip
        export ROS_HOSTNAME=$ip
        ```
    ```
    
    ```





## Running MOMObot Capabilities
This section assumes knowledge of ROS and basic Linorobot packages.
This is because MOMObot is based heavily on the Linorobot stack.

### Startup Process
1. Ensure that all electronics are connected and turned on
    - Ensure that the electronics battery has sufficient charge or the LIDAR will fail to function
    - Check to ensure that the LIDAR ethernet cable is connected properly or LIDAR data will not be parsed to the MOMObot stack
    - Disengage the E-stop 
3. SSH into Momobot:
    `ssh <USERNAME>@10.21.132.80`
4. Begin running commands, make sure they're in individual terminals!
    - When a command says **MOMOBOT**, do it in a terminal that is SSHed into MOMObot, or from the MOMObot computer directly
    - When the command says **COMPUTER**, do it on a ground station computer

5. **MOMOBOT:** Start ROSCORE before anything else (in it's own terminal on the MOMObot computer / SSH terminal): `roscore`
    - It is helpful to run `roscore` FIRST, so if the rest of the roslaunches die, you can kill and restart them easily without killing the ROS Master
6. **MOMOBOT:** Bringup the MOMObot base controller: `roslaunch momobot bringup.launch`
    - Ensure that the robot is not moving during this process as the IMU will be calibrating during bringup.launch. IMU drift will be present if the robot is in motion during this process
    - Encoder ticks will be visible in this terminal window. This can be a clear indication that the motor hall sensors are being detected and can also be used for the purposes of PID tuning
7. **COMPUTER:** Start the tele operation package in it's own terminal (preferably on the ground station computer, not MOMObot): 
    `rosrun teleop_twist_keyboard teleop_twist_keyboard`
    - This will enable tele-operation functionality on MOMObot (follow the screen fo instructions)
    
### Checking MOMO Functionality
1. **COMPUTER**: open rviz:
    - `roscd linorobot/rviz/`
    - `rviz -d odometry.rviz`
    - Under the topic tab on the left menu, it is possible to change the topic between `/odom` and `/raw_odom`.
    - It is also possible to add an additional topic and then set the keep to a larger number (e.g 10000) so as to allow for easier visualization
### Debugging
If visualization of all ROS nodes is required for debugging purposes, run 
     `rosrun rqt_graph rqt_graph`
     - This will bringup the rqt graph that visualizes all ROS relations and nodes, allowing for easy debugging.

![](https://i.imgur.com/6priv7s.png)

#### ROS nodes as seen in RQT Graph
- `/scan`: laser data
- `/laser_filter`: custom node for filtering laser data
- `/rosserial_lino`: adapter for connecting to teensy
    - It obtains `raw_vel` which is then sent to the `momo_base_node`
    - This information is then sent to `/raw_odom` for odometry estimations
    - `/raw_odom` information will be sent to `/ekf_localization` to be used for Extended Kalman Filtering estimation of the Robot's location
- `/imu/data`: the raw data from IMU
    - It is also fed into`/ekf_localization` 

After these commands are ran the following features on Momobot are enabled:
1. Teleop, motors
2. Laser data
3. Encoder and IMU
4. Odometry testing


#### Tuning of EKF
##### Method
1. Mark out a square of 3m x 3m using tape
2. Using tele-operation, drive the robot in a square
3. on **Computer** in the same terminal
    - `roscd/linorobot/rviz/`
    - `rviz -d odometry.rviz`
    - This can be used to view the distance the robot has think it has travelled
    - In Rviz
        1. Click on Odometry in the left topic pane (it is possible to select /odom and /raw_odom)
        2. It is recommended to add another topic and indicator for /raw_odom separately to visualize it easily
        3. Change the shaft length, radius and etc as necessary (set the color to 0, 255, 0 to ensure that the arrows are distinguishable from /odom arrows)
        4. Set laserscan to /scan_unfiltered to turn it off

4. Run the Navigation stack
    - `roslaunch momobot navigate.launch`


![](https://i.imgur.com/eSnwuAu.png)

`AMCL` is the navigator
`map_server` publishes the map

Autonomous localization and Navigation capabilities enabled and we can tell Momo to move by Rviz or by the command line:
1. Publishing to topics
     - publish to topics:
    `/move_base/goal`
    `/move_base_simple/goal`
2. RViz which is the easiest
    - Get your data by subscribing to those topics above using `rostopic echo <TOPIC_NAME>`


### Commands

Note: Each command is in each individual terminal, opened INSIDE MOMOBOT (i.e. in a terminal that is SSHed into MOMOBOT)


**Set Pose**
```shell
echo resetting pose... && rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 76.401, y: -15.676, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.10356, w: 0.99462}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}' --once && echo clearing costmaps... && rosservice call /move_base/clear_costmaps && echo done!
```

**Set Goal Pose for Autonomous Navigation** 
```shell
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.93851232529, y: -0.423947900534, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.125194964757, w: 0.992132158938}}}' --once && rosservice call /move_base/clear_costmaps
```

**Computer** load up rviz map again on your costmap
`roscd/linorobot/rviz/`
`rviz -d odometry.rviz`

**Note:** important to clear costmaps frequently as there will be phantom obstacles created the longer MOMO is in operation.

### To use SLAM
- `roslaunch momobot slam.launch` on **COMPUTER**
- `rosrun map_server map_saver` to save the map in the current directory. This saves your map files into .yaml files and .pgm files. .pgm can be editted in photoshop, etc like a .png file.
- if you need to change the map name, do it in the .yaml file.
**Setting your maps**
- under `roscd momobot`,, name the map properly as the .yaml files will be used to load map files here.

## Export Display to MOMObot from Computer
- `export DISPLAY:0` enables you to run commands on other computer instead of your computer. Ensure that you are on the right terminal **MOMOBOT**
- `rosrun momo_emotions cmd_vel_face_tracking.py` to run face cmd_vel tracking script on **MOMObot** from the **Computer Terminal**
- This allows the script to be started during events or situations where it may be difficult to access the on-board computer 

### Semantic Pose Package
- Must be on **MOMOBOT** `roslaunch semantic_pose_sounder semantic_pose_sound.launch`
- This loads all the ros parameters required.
- Edit them in the associated `config.yaml` in the package (Check it out with `roscd semantic_pose_sounder`)
- The location parameters are set in the following format: `'location: [boundingpt1, boundingpt2, boundingpt3...]'
    - Set as many bounding points as necessary but **ensure that they are in clockwise or anti-clockwise direction**
    - Bounding points define the boundary for a location (robot will be considered in that location if it is within the boundary)
    - Get bounding points from the map by running navigate.launch. The coordinates for various points on the map can be read easily
    - You can then define their names, which will be the strings published to `/location` when the robot is localised within those map zones
    - You can also define the corresponding MP3s to play!
- To check MOMObot's current location: `rostopic echo location`

### Tuning Navigation Parameters
- `roscd momobot` to cd into the momobot stack
- `roscd momobot/param/` change these files for navigation parameters

### Tuning Localization Parameters
- `roscd momobot/launch/include`, then `nano amcl.launch`
    - Parameters of interest:
        - laser_max_range
        - min_particles
        - max_particles
        - odom_alpha1 (Rotation noise from rotation)
        - odom_alpha2 (Rotation noise from translation)
        - odom_alpha3 (Translation noise from translation)
        - odom_alpha4 (Translation noise from rotation)

### Software To-Dos
- Laser scan matcher
- Visual odometry
- 3D depth camera integration
- Rear LiDAR integration (Need to combine the /scans somehow)
    - Once that's done we can swap to the DWA planner to allow MOMObot to reverse!
