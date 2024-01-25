# 配置
```bash
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc

```


## 本机编队导航

![](image/image_SSLmzaVWZw.png)

![](image/image_qzTJqy8uoN.png)

```bash
roslaunch turtlebot_formation2 All_multi_turtlebot3.launch 
roslaunch turtlebot_formation2 Leader_navigation.launch 
roslaunch turtlebot_formation2 col.launch 

```

### 六边形编队

```bash
roslaunch turtlebot_formation2 All5_multi_turtlebot3.launch 
roslaunch turtlebot_formation2 hexagon.launch 
#遥控
ROS_NAMESPACE=tb3_0 rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

```
### 三角形编队



编队坐标修改

```bash
<group if="$(eval multi_mode=='hexagon')">
    <node pkg="tf" type="static_transform_publisher" name="base_to_slave1" args="-1 -1 0 0 0 0   $(arg first_tb3) slave1 100" />
    <node pkg="tf" type="static_transform_publisher" name="base_to_slave2" args="-1  1 0 0 0 0   $(arg first_tb3) slave2 100" />
    <node pkg="tf" type="static_transform_publisher" name="base_to_slave3" args="-2  -1 0 0 0 0   $(arg first_tb3) slave3 100" />
    <node pkg="tf" type="static_transform_publisher" name="base_to_slave4" args="-2  1 0 0 0 0   $(arg first_tb3) slave4 100" />
    <node pkg="tf" type="static_transform_publisher" name="base_to_slave5" args="-3  0 0 0 0 0   $(arg first_tb3) slave5 100" />
</group>
```

## todo
```bash
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  151 (GLX)
  Minor opcode of failed request:  3 (X_GLXCreateContext)
  Value in failed request:  0x0
  Serial number of failed request:  31
  Current serial number in output stream:  32

```


```
File "/home/lwm/vscode\_ros/src/turtlebot\_formation2/scripts/laserTracker.py", line 13, in \<module>

scan=rospy.get\_param('\~scan')   &#x20;

File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/client.py", line 467, in get\_param

return \_param\_server\[param\_name] #MasterProxy does all the magic for us

File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/msproxy.py", line 123, in **getitem**

raise KeyError(key)

KeyError: '\~scan'
```
```bash
[ERROR] [1652417057.466797483, 0.341000000]: Tried to advertise a service that is already advertised in this node [/tb3_0/imu_service]

```

