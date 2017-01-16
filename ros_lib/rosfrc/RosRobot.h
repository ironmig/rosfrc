#ifndef SRC_ROSROBOT_H_
#define SRC_ROSROBOT_H_
#define BUILD_LIBROSSERIALEMBEDDEDLINUX

// Ros stuff
#include "ros.h"
#include <sensor_msgs/Joy.h>
#include <rosfrc/DriverStationStatus.h>
#include <rosfrc/Encoder.h>
#include <rosfrc/Gyro.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Trigger.h>

// FRC stuff
#include <DriverStation.h>
#include <IterativeRobot.h>
#include <HAL/HAL.h>
#include <LiveWindow/LiveWindow.h>
#include <networktables/NetworkTable.h>
#include <SpeedController.h>
#include <Joystick.h>
#include <Encoder.h>
#include <interfaces/Accelerometer.h>
#include <interfaces/Gyro.h>

// stdlib stuff
#include <memory>
#include <functional>

namespace rosfrc
{
	class Updater
	{
	public:
		virtual void update() = 0;
		virtual ~Updater() {};
	};
	class Joystick : public Updater
	{
	private:
		std::shared_ptr<frc::Joystick> joystick;
		ros::Publisher pub;
		sensor_msgs::Joy joy_msg;
	public:
		void update() override;
		Joystick (ros::NodeHandle& nh, const char* topic, frc::Joystick* joystick);
		Joystick (ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Joystick> joystick);
		virtual ~Joystick();
	};
	class DriverStation : public Updater
	{
	private:
		frc::DriverStation& ds;
		ros::Publisher pub;
		rosfrc::DriverStationStatus status_msg;
	public:
		void update() override;
		DriverStation (ros::NodeHandle& nh, const char* topic="/driver_station_status");
		virtual ~DriverStation();
	};
	class SpeedController : public Updater
	{
	private:
		std::shared_ptr<frc::SpeedController> controller;
		ros::Publisher feedback_pub;
		ros::Subscriber<std_msgs::Float64> goal_sub;
		std_msgs::Float64 feedback_msg;
	public:
		void update() override;
		void goal_cb(const std_msgs::Float64& data);
		SpeedController(ros::NodeHandle& nh, const char* topic, frc::SpeedController* controller);
		SpeedController(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::SpeedController> controller);
		virtual ~SpeedController();
	};
	class EncoderUpdater : public Updater
	{
	private:
		std::shared_ptr<frc::Encoder> m_encoder;
		ros::Publisher pub;
		ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> reset_server;
		rosfrc::Encoder encoder_msg;
	public:
		EncoderUpdater(ros::NodeHandle& nh, const char* topic, frc::Encoder* encoder);
		EncoderUpdater(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> encoder);
		void update() override;
	};
	class Accelerometer : public Updater
	{
	private:
		std::shared_ptr<frc::Accelerometer> m_accelerometer;
		ros::Publisher pub;
		geometry_msgs::Vector3 accel_msg;
	public:
		Accelerometer(ros::NodeHandle& nh, const char* topic, frc::Accelerometer* accelerometer);
		Accelerometer(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Accelerometer> accelerometer);
		void update() override;
	};
	class GyroUpdater : public Updater
	{
		std::shared_ptr<frc::Gyro> m_gyro;
		ros::Publisher pub;
		rosfrc::Gyro gyro_msg;
	public:
		GyroUpdater(ros::NodeHandle& nh, const char* topic, frc::Gyro* gyro);
		GyroUpdater(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Gyro> gyro);
		void update() override;
	};
}

class RosRobot : public frc::IterativeRobot {
private:
	ros::NodeHandle  nh;
	char* portName;
	std::vector<std::unique_ptr<rosfrc::Updater>> updaters;
	bool m_disabledInitialized = false;
	bool m_autonomousInitialized = false;
	bool m_teleopInitialized = false;
	bool m_testInitialized = false;
public:
	RosRobot(char* port);
	void StartCompetition() override;
	ros::NodeHandle& getRosNodeHandle();
	void AddJoystick(const char* name,std::shared_ptr<frc::Joystick> joystick);
	void AddSpeedController(const char* name, std::shared_ptr<frc::SpeedController>);
	void AddEncoder(const char* topic, std::shared_ptr<frc::Encoder> encoder);
	void AddAccelerometer(const char* topic, std::shared_ptr<frc::Accelerometer> accelerometer);
	void AddGyro(const char* topic, std::shared_ptr<frc::Gyro> gyro);
	void AddUpdater(rosfrc::Updater* updater);
	virtual ~RosRobot();
};
#endif /* SRC_ROSROBOT_H_ */
