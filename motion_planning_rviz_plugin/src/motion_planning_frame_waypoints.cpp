/******************************************************************
tab of waypoints for MoveIt planning under ROS 1

Features:
- add/remove waypoints
- plan and execute the trajectory of waypoints in Cartesian
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_waypoints.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace moveit_rviz_plugin
{
MotionPlanningFrameWaypointsWidget::MotionPlanningFrameWaypointsWidget(MotionPlanningDisplay* display, QWidget* parent)
  : QWidget(parent), ui_(new Ui::MotionPlanningFrameWaypointsUI()), planning_display_(display)
{
	std::cout << "\nWHI motion planning waypoints tab VERSION 00.05" << std::endl;
	std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	ui_->setupUi(this);

	// WHI logo
	struct passwd* pw = getpwuid(getuid());
	QImage logo;
	if (logo.load(QString(pw->pw_dir) + "/catkin_workspace/src/visualization/motion_planning_rviz_plugin/src/icons/whi_logo.png"))
	{
		QImage scaled = logo.scaledToHeight(48);
		ui_->logo_lable->setPixmap(QPixmap::fromImage(scaled));
	}

	// widget behaviour
	QStringList header = { "x", "y", "z", "roll", "pitch", "yaw" };
	ui_->waypoints_table->setColumnCount(header.size());
	ui_->waypoints_table->setHorizontalHeaderLabels(header);
	ui_->waypoints_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
	//ui_->waypoints_table->horizontalHeader()->setStretchLastSection(true);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (0)"));

	// signal
	connect(ui_->plan_button, &QPushButton::clicked, this, [=]() { planButtonClicked(); });
	connect(ui_->execute_button, &QPushButton::clicked, this, [=]() { executeButtonClicked(); });
	connect(ui_->plan_and_execute_button, &QPushButton::clicked, this, [=]() { planAndExecuteButtonClicked(); });
	connect(ui_->stop_button, &QPushButton::clicked, this, [=]() { stopButtonClicked(); });
	connect(ui_->add_point_button, &QPushButton::clicked, this, [=]() { addButtonClicked(); });
	connect(ui_->insert_point_button, &QPushButton::clicked, this, [=]() { insertButtonClicked(); });
	connect(ui_->remove_point_button, &QPushButton::clicked, this, [=]() { removeButtonClicked(); });
	connect(ui_->waypoints_table, &QTableWidget::cellChanged, this, [=]() { visualizeWaypoints(); });
}

MotionPlanningFrameWaypointsWidget::~MotionPlanningFrameWaypointsWidget()
{
	delete ui_;
}

void MotionPlanningFrameWaypointsWidget::setPlanningGroupName(const QString& Name)
{
	ui_->planning_group_name->setText(Name);
}

void MotionPlanningFrameWaypointsWidget::configureForPlanning(moveit::planning_interface::MoveGroupInterfacePtr MoveGroup)
{
	MoveGroup->setStartState(*planning_display_->getQueryStartState());
	MoveGroup->setJointValueTarget(*planning_display_->getQueryGoalState());
	MoveGroup->setPlanningTime(ui_->planning_time->value());
	MoveGroup->setNumPlanningAttempts(ui_->planning_attempts->value());
	MoveGroup->setMaxVelocityScalingFactor(ui_->velocity_scaling_factor->value());
	MoveGroup->setMaxAccelerationScalingFactor(ui_->acceleration_scaling_factor->value());
}

void MotionPlanningFrameWaypointsWidget::registerPlan(const PoseUiCallback& Func)
{
	func_plan_ = Func;
}

void MotionPlanningFrameWaypointsWidget::registerExecute(const UiCallback& Func)
{
	func_execute_ = Func;
}

void MotionPlanningFrameWaypointsWidget::registerPlanAndExecute(const PoseUiCallback& Func)
{
	func_plan_and_execute_ = Func;
}

void MotionPlanningFrameWaypointsWidget::registerStop(const NullCallback& Func)
{
	func_stop_ = Func;
}

void MotionPlanningFrameWaypointsWidget::planButtonClicked()
{
	std::vector<geometry_msgs::Pose> waypoints;
	getWaypoints(waypoints);

	if (func_plan_)
	{
		func_plan_(waypoints, ui_);
	}
}

void MotionPlanningFrameWaypointsWidget::executeButtonClicked()
{
	if (func_execute_)
	{
		if (ui_->looping->isChecked())
		{
			std::thread{ [=]()
			{
				while (ui_->looping->isChecked())
				{
					func_execute_(ui_);
					
					do
					{					
						std::this_thread::sleep_for(std::chrono::milliseconds(500));
					} while (ui_->stop_button->isEnabled());
				}
			} }.detach();
		}
		else
		{
			func_execute_(ui_);
		}
	}
}

void MotionPlanningFrameWaypointsWidget::planAndExecuteButtonClicked()
{
	std::vector<geometry_msgs::Pose> waypoints;
	getWaypoints(waypoints);

	if (func_plan_and_execute_)
	{
		func_plan_and_execute_(waypoints, ui_);
	}
}

void MotionPlanningFrameWaypointsWidget::stopButtonClicked()
{
	if (func_stop_)
	{
		func_stop_();
	}
}

void MotionPlanningFrameWaypointsWidget::addButtonClicked()
{
	ui_->waypoints_table->blockSignals(true);
	ui_->waypoints_table->insertRow(ui_->waypoints_table->rowCount());
	for (int i = 0; i < ui_->waypoints_table->columnCount(); ++i)
	{
		ui_->waypoints_table->setItem(ui_->waypoints_table->rowCount() - 1, i, new QTableWidgetItem("0.0"));
	}
	ui_->waypoints_table->blockSignals(false);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	visualizeWaypoints();
}

void MotionPlanningFrameWaypointsWidget::insertButtonClicked()
{
	ui_->waypoints_table->blockSignals(true);
	ui_->waypoints_table->insertRow(ui_->waypoints_table->currentRow());
	for (int i = 0; i < ui_->waypoints_table->columnCount(); ++i)
	{
		ui_->waypoints_table->setItem(ui_->waypoints_table->currentRow() - 1, i, new QTableWidgetItem("0.0"));
	}
	ui_->waypoints_table->blockSignals(false);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	visualizeWaypoints();
}

void MotionPlanningFrameWaypointsWidget::removeButtonClicked()
{
	ui_->waypoints_table->removeRow(ui_->waypoints_table->currentRow());
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	visualizeWaypoints();
}

void MotionPlanningFrameWaypointsWidget::getWaypoints(std::vector<geometry_msgs::Pose>& Waypoints)
{
  	for (int i = 0; i < ui_->waypoints_table->rowCount(); ++i)
	{
		geometry_msgs::Pose pose;
		pose.position.x = ui_->waypoints_table->item(i, 0)->text().toDouble();
		pose.position.y = ui_->waypoints_table->item(i, 1)->text().toDouble();
		pose.position.z = ui_->waypoints_table->item(i, 2)->text().toDouble();

		tf2::Quaternion orientation;
		orientation.setRPY(angles::from_degrees(ui_->waypoints_table->item(i, 3)->text().toDouble()),
			angles::from_degrees(ui_->waypoints_table->item(i, 4)->text().toDouble()),
			angles::from_degrees(ui_->waypoints_table->item(i, 5)->text().toDouble()));

		pose.orientation.x = orientation.getX();
		pose.orientation.y = orientation.getY();
		pose.orientation.z = orientation.getZ();
		pose.orientation.w = orientation.getW();
#ifdef DEBUG
		std::cout << "orientation " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << std::endl;
#endif

		Waypoints.push_back(pose);
	}
}

void MotionPlanningFrameWaypointsWidget::visualizeWaypoints()
{
	std::vector<geometry_msgs::Pose> waypoints;
	getWaypoints(waypoints);

	std::vector<geometry_msgs::PoseStamped> stamped;
	for (const auto& it : waypoints)
	{
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.pose = it;
		stamped.push_back(poseStamped);
	}
	planning_display_->visualizeWaypointsLocations(stamped);
}
}  // namespace moveit_rviz_plugin

