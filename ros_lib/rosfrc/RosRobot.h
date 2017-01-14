#ifndef SRC_ROSROBOT_H_
#define SRC_ROSROBOT_H_
#define BUILD_LIBROSSERIALEMBEDDEDLINUX
#include <IterativeRobot.h>
#include "ros.h"
#include <sensor_msgs/Joy.h>
#include <rosfrc/DriverStationStatus.h>
#include <rosfrc/Encoder.h>
#include <std_msgs/Float64.h>

#include "DriverStation.h"
#include "HAL/HAL.h"
#include "LiveWindow/LiveWindow.h"
#include "networktables/NetworkTable.h"
#include <SpeedController.h>
#include <Joystick.h>
#include <Encoder.h>

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
		rosfrc::Encoder encoder_msg;
	public:
		EncoderUpdater(ros::NodeHandle& nh, const char* topic, frc::Encoder* encoder);
		EncoderUpdater(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> encoder);
		void update() override;
	};
}

class RosRobot : frc::IterativeRobot {
private:
	ros::NodeHandle  nh;

	std::vector<std::unique_ptr<rosfrc::Updater>> updaters;
	bool m_disabledInitialized = false;
	bool m_autonomousInitialized = false;
	bool m_teleopInitialized = false;
	bool m_testInitialized = false;
public:
	RosRobot(char* portName);
	void StartCompetition() override;
	ros::NodeHandle& getRosNodeHandle();
	void AddJoystick(const char* name,std::shared_ptr<frc::Joystick> joystick);
	void AddSpeedController(const char* name, std::shared_ptr<frc::SpeedController>);
	virtual ~RosRobot();
};
#endif /* SRC_ROSROBOT_H_ */
