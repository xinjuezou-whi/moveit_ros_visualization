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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <thread>
#include <unistd.h>
#include <QtWidgets/QMenu>

namespace moveit_rviz_plugin
{
MotionPlanningFrameWaypointsWidget::MotionPlanningFrameWaypointsWidget(MotionPlanningDisplay* display, QWidget* parent)
  : QWidget(parent), ui_(new Ui::MotionPlanningFrameWaypointsUI()), planning_display_(display)
{
	std::cout << "\nWHI motion planning waypoints tab VERSION 00.09" << std::endl;
	std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	ui_->setupUi(this);

	// WHI logo
	boost::filesystem::path path(ros::package::getPath("moveit_ros_visualization"));
	QImage logo;
	if (logo.load(QString(path.string().c_str()) + "/motion_planning_rviz_plugin/src/icons/whi_logo.png"))
	{
		QImage scaled = logo.scaledToHeight(48);
		ui_->logo_label->setPixmap(QPixmap::fromImage(scaled));
	}

    // display
    planning_display_->registerWaypointUpdate(std::bind(&MotionPlanningFrameWaypointsWidget::updateWaypoint, this, std::placeholders::_1, std::placeholders::_2));

	// widget behaviour
	QStringList header = { "x", "y", "z", "roll", "pitch", "yaw" };
	ui_->waypoints_table->setColumnCount(header.size());
	ui_->waypoints_table->setHorizontalHeaderLabels(header);
	ui_->waypoints_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
	//ui_->waypoints_table->horizontalHeader()->setStretchLastSection(true);
	ui_->waypoints_table->setContextMenuPolicy(Qt::CustomContextMenu);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (0)"));

	// signal
	connect(ui_->plan_button, &QPushButton::clicked, this, [=]() { planButtonClicked(); });
	connect(ui_->execute_button, &QPushButton::clicked, this, [=]() { executeButtonClicked(); });
	connect(ui_->plan_and_execute_button, &QPushButton::clicked, this, [=]() { planAndExecuteButtonClicked(); });
	connect(ui_->stop_button, &QPushButton::clicked, this, [=]() { stopButtonClicked(); });
	connect(ui_->add_point_button, &QPushButton::clicked, this, [=]() { addButtonClicked(); });
	connect(ui_->insert_point_button, &QPushButton::clicked, this, [=]() { insertButtonClicked(); });
	connect(ui_->remove_point_button, &QPushButton::clicked, this, [=]() { removeButtonClicked(); });
	connect(ui_->waypoints_table, &QTableWidget::cellChanged, this, [=](int Row, int Column) { visualizeWaypoints(Row); });
    connect(ui_->waypoints_table, &QTableWidget::currentCellChanged, this, [=](int Row, int Column) { visualizeWaypoints(Row); });
	connect(ui_->waypoints_table, &QTableWidget::customContextMenuRequested, this, [=](const QPoint& Pos)
	{
        QMenu* menu = new QMenu;
        auto action = menu->addAction("current");
        QObject::connect(action, &QAction::triggered, this, [=]()
		{
			QTableWidgetItem* item = ui_->waypoints_table->itemAt(Pos);

			fillWaypoint(item->row(), true);

			action->deleteLater();
            menu->deleteLater();
        });
        menu->exec(QCursor::pos());
        menu->clear();
    });
}

MotionPlanningFrameWaypointsWidget::~MotionPlanningFrameWaypointsWidget()
{
	delete ui_;
}

void MotionPlanningFrameWaypointsWidget::setMoveGroup(moveit::planning_interface::MoveGroupInterfacePtr MoveGroup, const QString& Name)
{
	move_group_ = MoveGroup;
	ui_->planning_group_name->setText(Name);
}

void MotionPlanningFrameWaypointsWidget::configureForPlanning(moveit::planning_interface::MoveGroupInterfacePtr MoveGroup)
{
	move_group_ = MoveGroup;
	if (move_group_)
	{
		move_group_->setStartState(*planning_display_->getQueryStartState());
		move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
		move_group_->setPlanningTime(ui_->planning_time->value());
		move_group_->setNumPlanningAttempts(ui_->planning_attempts->value());
		move_group_->setMaxVelocityScalingFactor(ui_->velocity_scaling_factor->value());
		move_group_->setMaxAccelerationScalingFactor(ui_->acceleration_scaling_factor->value());
	}
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
					
    				// wait until execution finished
    				std::unique_lock<std::mutex> lk(mtx_);
    				cv_.wait(lk);

					std::this_thread::sleep_for(std::chrono::milliseconds(400));
					lk.unlock();
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
	fillWaypoint(ui_->waypoints_table->rowCount() - 1, ui_->current->isChecked());
	ui_->waypoints_table->blockSignals(false);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	ui_->waypoints_table->setCurrentCell(ui_->waypoints_table->rowCount() - 1, 0);
}

void MotionPlanningFrameWaypointsWidget::insertButtonClicked()
{
	ui_->waypoints_table->blockSignals(true);
	ui_->waypoints_table->insertRow(ui_->waypoints_table->currentRow());
	fillWaypoint(ui_->waypoints_table->currentRow() - 1, ui_->current->isChecked());
	ui_->waypoints_table->blockSignals(false);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	ui_->waypoints_table->setCurrentCell(ui_->waypoints_table->currentRow() - 1, 0);
}

void MotionPlanningFrameWaypointsWidget::removeButtonClicked()
{
	int highlightRow = ui_->waypoints_table->currentRow() == ui_->waypoints_table->rowCount() - 1 ? ui_->waypoints_table->rowCount() - 2 : ui_->waypoints_table->currentRow();
	ui_->waypoints_table->blockSignals(true);
	ui_->waypoints_table->removeRow(ui_->waypoints_table->currentRow());
	ui_->waypoints_table->blockSignals(false);
	ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->waypoints_table->rowCount()) + QString(")"));

	ui_->waypoints_table->setCurrentCell(highlightRow, 0);
	visualizeWaypoints(highlightRow);
}

void MotionPlanningFrameWaypointsWidget::fillWaypoint(int RowIndex, bool WithCurrent/* = false*/)
{
	if (WithCurrent && move_group_)
	{
		moveit::core::RobotState current = *planning_display_->getQueryStartState();
		const std::string& link_name = move_group_->getEndEffectorLink();
  		const moveit::core::LinkModel* link = move_group_->getRobotModel()->getLinkModel(link_name);

		geometry_msgs::Pose currentPose = tf2::toMsg(current.getGlobalLinkTransform(link));
		ui_->waypoints_table->setItem(RowIndex, 0, new QTableWidgetItem(QString::number(currentPose.position.x)));
		ui_->waypoints_table->setItem(RowIndex, 1, new QTableWidgetItem(QString::number(currentPose.position.y)));
		ui_->waypoints_table->setItem(RowIndex, 2, new QTableWidgetItem(QString::number(currentPose.position.z)));
		tf::Quaternion quat(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w);
  		double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		ui_->waypoints_table->setItem(RowIndex, 3, new QTableWidgetItem(QString::number(angles::to_degrees(roll))));
		ui_->waypoints_table->setItem(RowIndex, 4, new QTableWidgetItem(QString::number(angles::to_degrees(pitch))));
		ui_->waypoints_table->setItem(RowIndex, 5, new QTableWidgetItem(QString::number(angles::to_degrees(yaw))));
	}
	else
	{
		for (int i = 0; i < ui_->waypoints_table->columnCount(); ++i)
		{
			ui_->waypoints_table->setItem(RowIndex, i, new QTableWidgetItem("0.0"));
		}
	}
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

		pose.orientation = tf2::toMsg(orientation);
#ifdef DEBUG
		std::cout << "quaternion from euler " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << std::endl;
#endif

		Waypoints.push_back(pose);
	}
}

void MotionPlanningFrameWaypointsWidget::visualizeWaypoints(int Row)
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
	planning_display_->visualizeWaypointsLocations(Row, stamped);
}

void MotionPlanningFrameWaypointsWidget::updateWaypoint(int Index, const geometry_msgs::Pose& Pose)
{
	tf::Quaternion quat(Pose.orientation.x, Pose.orientation.y, Pose.orientation.z, Pose.orientation.w);
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	ui_->waypoints_table->blockSignals(true);
	ui_->waypoints_table->setItem(Index, 0, new QTableWidgetItem(QString::number(Pose.position.x)));
	ui_->waypoints_table->setItem(Index, 1, new QTableWidgetItem(QString::number(Pose.position.y)));
	ui_->waypoints_table->setItem(Index, 2, new QTableWidgetItem(QString::number(Pose.position.z)));
	ui_->waypoints_table->setItem(Index, 3, new QTableWidgetItem(QString::number(angles::to_degrees(roll))));
	ui_->waypoints_table->setItem(Index, 4, new QTableWidgetItem(QString::number(angles::to_degrees(pitch))));
	ui_->waypoints_table->setItem(Index, 5, new QTableWidgetItem(QString::number(angles::to_degrees(yaw))));
	ui_->waypoints_table->blockSignals(false);
}

void MotionPlanningFrameWaypointsWidget::mousePressEvent(QMouseEvent* Event)
{
	if (Event->button() == Qt::LeftButton)
	{
		if (ui_->waypoints_table->itemAt(Event->pos()) == nullptr)
		{
			visualizeWaypoints(-1);
		}
    }
}

void MotionPlanningFrameWaypointsWidget::notifyCv()
{
	cv_.notify_one();
}
}  // namespace moveit_rviz_plugin

