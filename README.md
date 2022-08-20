# moveit_ros_visualization
This is a Waypoints planning RViz plugin for MoveIt. I decided to create a tab page within "MotionPlanning" plugin rather then to create a separate panel is because the latter solution will take further more space of RViz's window. More over as a tab page, its implementation can take the advantages of reusing the existing facilities.

![image](https://user-images.githubusercontent.com/72239958/185739812-f6e00347-55e9-4fea-97fb-9e11d910a7a4.png)


With this tab, user can provide a set of points to plan a go-through trajectory with the Cartesian Space.
![plugin02](https://user-images.githubusercontent.com/72239958/185740142-1f53134a-ce2d-46ec-ba0d-558ae2f1d275.gif)

![plugin](https://user-images.githubusercontent.com/72239958/185739767-5041dca0-04c3-4a65-ba74-3a68cf683097.gif)

## Note
Currently, only the branch "melodic" is implemented. The main branch just takes a place for later implementation. 

## Usage
### MoveIt intalled from source
By this case, just clone this repository and replace the "visualization", then build.

### MoveIt intalled from binary
First, neutralize the installed libs: "libmoveit_motion_planning_rviz_plugin.so" and "libmoveit_motion_planning_rviz_plugin_core.so"
```
sudo mv /opt/ros/melodic/lib/libmoveit_motion_planning_rviz_plugin.so /opt/ros/melodic/lib/libmoveit_motion_planning_rviz_plugin.so.bk
sudo mv /opt/ros/melodic/lib/libmoveit_motion_planning_rviz_plugin_core.so /opt/ros/melodic/lib/libmoveit_motion_planning_rviz_plugin_core.so.bk
```

Then, clone branch "melodic" of this repository to the src folder of your workspace
```
cd <your_workspace>/src
git clone -b melodic https://github.com/xinjuezou-whi/moveit_ros_visualization.git
```

and build
```
cd ..
catkin build
```
or
```
catkin_make
```
depends on your environment.

Once the build is finished, the tab "Waypoints" will be loaded following the "Planning" while running the MoveIt launch file. Let's take the panda_moveit_config as an example:
```
cd <your_workspace>
roslaunch panda_moveit_config demo.launch
```
![image](https://user-images.githubusercontent.com/72239958/185741656-eb68b2a2-c2f3-4f81-93ec-fd94e88ce453.png)

## Further revisement
1. save waypoints to file, and load them from saved file
2. catch the point from current state
3. ...