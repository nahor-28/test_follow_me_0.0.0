# Follow Me Mode Simulations

This repo contains packages of [**iq_sim**](https://github.com/Intelligent-Quads/iq_sim), [**iq_gnc**](https://github.com/Intelligent-Quads/iq_gnc) and [**darknet_ros**](https://github.com/leggedrobotics/darknet_ros) for testing the follow me mode we are trying to develop and test it on our drone. the codes here specifically are designed to run with the Ardupilot control system and utilizes the ardupilot gazebo plugin to allow the ardupilot control software to interface and control the model drone in gazebo.

***

## How to execute

First start the ***iq_sim*** launch file

```bash
roslaunch iq_sim runway.launch
```

Then add a person model in gazebo environment infront of the drone camera.
Then launch the ***darknet_ros*** launch file.

```bash
roslaunch darknet_ros darknet_ros.launch
```

Start the SITL using the *startsitl.sh* script or using the *sim_vehicle.py* file

```bash
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

```bash
roslaunch iq_sim apm.launch
```

Then run the ***track*** file

```bash
rosrun iq_gnc person_d
```


