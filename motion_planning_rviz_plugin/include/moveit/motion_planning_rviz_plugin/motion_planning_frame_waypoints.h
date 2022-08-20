/******************************************************************
tab of waypoints for MoveIt planning under ROS 1

Features:
- add/remove waypoints
- plan and execute the trajectory of waypoints in Cartesian
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-08-18: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <moveit/macros/class_forward.h>
#include <geometry_msgs/Pose.h>
#include <QWidget>
#include <vector>
#include <memory>

#include "ui_motion_planning_rviz_plugin_frame_waypoints.h"

namespace robot_interaction
{
MOVEIT_CLASS_FORWARD(InteractionHandler)
}

namespace moveit_rviz_plugin
{
// forward declaration
class MotionPlanningDisplay;

using PoseUiCallback = std::function<void(const std::vector<geometry_msgs::Pose>&, Ui::MotionPlanningFrameWaypointsUI*)>;
using UiCallback = std::function<void(Ui::MotionPlanningFrameWaypointsUI*)>;
using NullCallback = std::function<void(void)>;

class MotionPlanningFrameWaypointsWidget : public QWidget
{
  	Q_OBJECT

public:
  	MotionPlanningFrameWaypointsWidget(const MotionPlanningFrameWaypointsWidget&) = delete;
  	MotionPlanningFrameWaypointsWidget(MotionPlanningDisplay* display, QWidget* parent = nullptr);
  	~MotionPlanningFrameWaypointsWidget() override;

public:
	void setPlanningGroupName(const QString& Name);
	void registerPlan(const PoseUiCallback& Func);
	void registerExecute(const UiCallback& Func);
	void registerPlanAndExecute(const PoseUiCallback& Func);
	void registerStop(const NullCallback& Func);

private:
	void planButtonClicked();
	void executeButtonClicked();
	void planAndExecuteButtonClicked();
	void stopButtonClicked();
	void addButtonClicked();
	void insertButtonClicked();
	void removeButtonClicked();
    void getWaypoints(std::vector<geometry_msgs::Pose>& Waypoints);
    void visualizeWaypoints();

private:
  	Ui::MotionPlanningFrameWaypointsUI* ui_{ nullptr };
  	MotionPlanningDisplay* planning_display_{ nullptr };
	PoseUiCallback func_plan_{ nullptr };
	UiCallback func_execute_{ nullptr };
	PoseUiCallback func_plan_and_execute_{ nullptr };
	NullCallback func_stop_{ nullptr };
};
}  // namespace moveit_rviz_plugin

