# fv_avalanche_mvd

## simulation
Start the PX4 simulation:
```sh
cd ~/PX4-Autopilot/
make px4_sitl gz_x500
```
A gazebo window with the drone should appear. Now you can also launch [QGroundControll](http://qgroundcontrol.com) and control the drone already.

Start the DDS client in a new terminal:
```sh
MicroXRCEAgent udp4 -p 8888
```

Start the avalanche search in a new terminal:
```sh
~/ros2_ws
ros2 launch fv_avalanche_mvd  sim.launch.py 
```

ros2 launch fv_avalanche_mvd  sim.launch.py


## read WiFi
Start the wifi publisher:

There are two possibilities. either with ```iwlist``` (must be installed) or with ```pywifi``` ( ```pip install pywifi``` ) in the ```wifi.launch.py``` file the corresponding ```executable='read_wifi_pyWiFi', ``` or ```executable='read_wifi',``` file must be commented out.

```sh
~/ros2_ws
rosrun ros2 launch fv_avalanche_mvd wifi.launch.py ssid:=YOUR_SSID
```