# moveit_ros_visualization
This is a Waypoints planning RViz plugin for MoveIt. I decided to create a tab page within "MotionPlanning" plugin rather then to create a separate panel is because the latter solution will take further more space of RViz's window. More over as a tab page, its implementation can take the advantages of reusing the existing facilities.

![image](https://user-images.githubusercontent.com/72239958/185739812-f6e00347-55e9-4fea-97fb-9e11d910a7a4.png)

With this tab, user can provide a set of points to plan a go-through trajectory with the Cartesian Space.

![add](https://user-images.githubusercontent.com/72239958/187058845-9a0351a2-b155-47d7-aca6-474426c4ae6b.gif)

![plan](https://user-images.githubusercontent.com/72239958/187059205-af3d3d56-36ac-4211-b69f-c6d765478152.gif)


## Note
Currently, the branch "main" has not been tested yet.

## Refactor and Implementation
| File                                                                  | Plugin          | Class    | Comment                                                                                                    |
|-----------------------------------------------------------------------|-----------------|----------|------------------------------------------------------------------------------------------------------------|
| motion_planning_display.h motion_planning_display.cpp                 | motion_planning | refactor | added visualize function for waypoints                                                                     |
| motion_planning_frame.h motion_planning_frame.cpp                     | motion_planning | refactor | forward declaration of tab waypoints added signal callback of tab waypoints overrided computeCartesianPlan |
| motion_planning_frame_planning.cpp                                    | motion_planning | refactor | overrided computeCartesianPlan                                                                             |
| CMakeLists.txt                                                        | motion_planning | refactor | new file's manifest                                                                                        |
| motion_planning_frame_waypoints.h motion_planning_frame_waypoints.cpp | motion_planning | new      | properties and behaviors of widgets of tab waypoints ux logic                                              |
| motion_planning_rviz_plugin_frame_waypoints.ui                        | motion_planning | new      | ui from QT Designer                                                                                        |
| whi_logo.png                                                          | motion_planning | new      | logo image                                                                                                 |

## Build
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
depends on your environment. And do not forget to source your workspace
```
source <your_workspace>/devel/setup.bash
```

## Usage
Once the build is finished, the tab "Waypoints" will be loaded following the "Planning" while running the MoveIt launch file. Let's take the panda_moveit_config as an example:
```
cd <your_workspace>
roslaunch panda_moveit_config demo.launch
```

![image](https://user-images.githubusercontent.com/72239958/187059302-82aaa6de-dfbc-48b5-bc96-2e7af905b2d3.png)


### Loop execution
For a real-world arm, given the plan operation is succeed, check the "Loop Execution" and then execute, the arm will repeat the planned trajectory.

![plugin03](https://user-images.githubusercontent.com/72239958/185743085-e0892db0-76ad-49d9-8e7f-62e9640f1486.gif)

### Point from current state
There are two ways to add a point from current state of eef. One is to check the "current" before click "Add" or "Insert" button. And the other is to right-click the added point in waypoints list, then click "current" on pop-up menu.

![current](https://user-images.githubusercontent.com/72239958/187059724-b785d26f-bd3f-4a3e-bb08-549417426927.gif)


## Further revisement
1. save waypoints to file, and load them from saved file
2. ...
