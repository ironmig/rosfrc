#include "RosRobot.h"

rosfrc::RosRobot::RosRobot(char* port) :
	portName(port)
{
  nh.initNode(portName);
}

rosfrc::RosRobot::~RosRobot() {

}

ros::NodeHandle& rosfrc::RosRobot::getNodeHandle()
{
	return nh;
}

void rosfrc::RosRobot::StartCompetition() {
  HAL_Report(HALUsageReporting::kResourceType_Framework,
             HALUsageReporting::kFramework_Iterative);
  LiveWindow* lw = LiveWindow::GetInstance();
  // first and one-time initialization
  NetworkTable::GetTable("LiveWindow")
      ->GetSubTable("~STATUS~")
      ->PutBoolean("LW Enabled", false);
  RobotInit();

  // Tell the DS that the robot is ready to be enabled
  HAL_ObserveUserProgramStarting();

  // loop forever, calling the appropriate mode-dependent function
  lw->SetEnabled(false);

  updaters.push_back(std::unique_ptr<rosfrc::Updater>(new rosfrc::DriverStation(nh, "/ds_update")));
  while (true) {
	nh.spinOnce();
	if (nh.connected())
	{
		// Update all the ros updaters (DriverStation, Joysticks, etc)
		for (size_t i = 0; i < updaters.size(); i++)
			updaters[i]->update();
	} else nh.initNode(portName);
    // wait for driver station data so the loop doesn't hog the CPU
    if(!m_ds.WaitForData(0.1))
    {
        RobotPeriodic();
    	continue;
    }
    // Call the appropriate function depending upon the current robot mode
    if (IsDisabled()) {
      // call DisabledInit() if we are now just entering disabled mode from
      // either a different mode or from power-on
      if (!m_disabledInitialized) {
        lw->SetEnabled(false);
        DisabledInit();
        m_disabledInitialized = true;
        // reset the initialization flags for the other modes
        m_autonomousInitialized = false;
        m_teleopInitialized = false;
        m_testInitialized = false;
      }
      HAL_ObserveUserProgramDisabled();
      DisabledPeriodic();
    } else if (IsAutonomous()) {
      // call AutonomousInit() if we are now just entering autonomous mode from
      // either a different mode or from power-on
      if (!m_autonomousInitialized) {
        lw->SetEnabled(false);
        AutonomousInit();
        m_autonomousInitialized = true;
        // reset the initialization flags for the other modes
        m_disabledInitialized = false;
        m_teleopInitialized = false;
        m_testInitialized = false;
      }
      HAL_ObserveUserProgramAutonomous();
      AutonomousPeriodic();
    } else if (IsTest()) {
      // call TestInit() if we are now just entering test mode from
      // either a different mode or from power-on
      if (!m_testInitialized) {
        lw->SetEnabled(true);
        TestInit();
        m_testInitialized = true;
        // reset the initialization flags for the other modes
        m_disabledInitialized = false;
        m_autonomousInitialized = false;
        m_teleopInitialized = false;
      }
      HAL_ObserveUserProgramTest();
      TestPeriodic();
    } else {
		
      // call TeleopInit() if we are now just entering teleop mode from
      // either a different mode or from power-on
      if (!m_teleopInitialized) {
        lw->SetEnabled(false);
        TeleopInit();
        m_teleopInitialized = true;
        // reset the initialization flags for the other modes
        m_disabledInitialized = false;
        m_autonomousInitialized = false;
        m_testInitialized = false;
        Scheduler::GetInstance()->SetEnabled(true);
      }
      HAL_ObserveUserProgramTeleop();
      TeleopPeriodic();
    }
    RobotPeriodic();
  }
}
void rosfrc::RosRobot::AddUpdater(rosfrc::Updater* updater)
{
	updaters.push_back(std::unique_ptr<rosfrc::Updater>(updater));
}
void rosfrc::RosRobot::AddJoystick(const char* name, std::shared_ptr<frc::Joystick> stick)
{
	AddUpdater(new rosfrc::Joystick(nh, name, stick));
}
void rosfrc::RosRobot::AddSpeedController(const char* name, std::shared_ptr<frc::SpeedController> controller)
{
	AddUpdater(new rosfrc::SpeedController(nh, name, controller));
}
void rosfrc::RosRobot::AddEncoder(const char* topic, std::shared_ptr<frc::Encoder> encoder)
{
	AddUpdater(new rosfrc::EncoderUpdater(nh, topic, encoder));
}
void rosfrc::RosRobot::AddAccelerometer(const char* topic, std::shared_ptr<frc::Accelerometer> accelerometer)
{
	AddUpdater(new rosfrc::Accelerometer(nh, topic, accelerometer));
}
void rosfrc::RosRobot::AddGyro(const char* topic, std::shared_ptr<frc::Gyro> gyro)
{
	AddUpdater(new rosfrc::GyroUpdater(nh, topic, gyro));
}

rosfrc::Joystick::Joystick(ros::NodeHandle& nh, const char* topic,frc::Joystick* stick) :
		rosfrc::Joystick(nh, topic, std::shared_ptr<frc::Joystick> (stick))
{

}
rosfrc::Joystick::Joystick (ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Joystick> stick):
		joystick(stick),
		pub(topic, &joy_msg)
{
	nh.advertise(pub);
}
void rosfrc::Joystick::update()
{
	if (joy_msg.axes_length != joystick->GetAxisCount())
	{
		joy_msg.axes_length = joystick->GetAxisCount();
		delete[] joy_msg.axes;
		joy_msg.axes = new float[joy_msg.axes_length];
	}
	if (joy_msg.buttons_length != joystick->GetButtonCount())
	{
		joy_msg.buttons_length = joystick->GetButtonCount();
		delete[] joy_msg.buttons;
		joy_msg.buttons= new int32_t[joy_msg.buttons_length];
	}
	for (size_t i = 0; i < joy_msg.axes_length; i++)
		joy_msg.axes[i] = joystick->GetRawAxis(i);
	for (size_t i = 1; i <= joy_msg.buttons_length; i++)
		joy_msg.buttons[i-1] = joystick->GetRawButton(i);

	pub.publish(&joy_msg);
}
rosfrc::Joystick::~Joystick()
{

}

rosfrc::DriverStation::DriverStation(ros::NodeHandle& nh, const char* topic) :
		ds(frc::DriverStation::GetInstance()),
		pub(topic, &status_msg)
{
	nh.advertise(pub);
}
void rosfrc::DriverStation::update()
{
	//set msg status
	if (ds.IsDisabled()) status_msg.status = "DISABLED";
	else if (ds.IsAutonomous()) status_msg.status = "AUTO";
	else if (ds.IsTest()) status_msg.status = "TEST";
	else status_msg.status = "OPERATORONTROL";

	//set alliance
	frc::DriverStation::Alliance alliance = ds.GetAlliance();
	if (alliance == frc::DriverStation::Alliance::kBlue) status_msg.ALLIANCE = "BLUE";
	else if (alliance == frc::DriverStation::Alliance::kRed) status_msg.ALLIANCE = "RED";
	else status_msg.ALLIANCE = "INVALID";

	status_msg.LOCATION = ds.GetLocation();
	status_msg.attached = ds.IsDSAttached();
	status_msg.fms = ds.IsFMSAttached();
	status_msg.matchTime.fromSec(ds.GetMatchTime());

	pub.publish(&status_msg);
}
rosfrc::DriverStation::~DriverStation()
{

}

void rosfrc::SpeedController::goal_cb(const std_msgs::Float64& data)
{

}
rosfrc::SpeedController::SpeedController(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::SpeedController> control):
		controller(control),
		feedback_pub((new std::string(topic+std::string("/feedback")))->c_str(), &feedback_msg),
		goal_sub((new std::string(topic+std::string("/goal")))->c_str(), [this] (const std_msgs::Float64& data) {
			controller->Set(data.data);
		})
{
       nh.advertise(feedback_pub);
       nh.subscribe(goal_sub);

}
rosfrc::SpeedController::SpeedController(ros::NodeHandle& nh, const char* topic, frc::SpeedController* control):
		rosfrc::SpeedController(nh, topic, std::shared_ptr<frc::SpeedController>(control))
{

}
void rosfrc::SpeedController::update()
{
	feedback_msg.data = controller->Get();
	feedback_pub.publish(&feedback_msg);
}
rosfrc::SpeedController::~SpeedController()
{

}

rosfrc::EncoderUpdater::EncoderUpdater(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> encoder) :
	m_encoder(encoder),
	pub(topic, &encoder_msg),
	reset_server((new std::string(topic+std::string("/reset")))->c_str(), [this] (const std_srvs::Trigger::Request req, std_srvs::Trigger::Response res) {
		m_encoder->Reset();
		res.success = true;
	})
{
	nh.advertise(pub);
	nh.advertiseService(reset_server);
}
rosfrc::EncoderUpdater::EncoderUpdater(ros::NodeHandle& nh, const char* topic, frc::Encoder* encoder) :
	rosfrc::EncoderUpdater(nh, topic, std::shared_ptr<frc::Encoder>(encoder))
{

}
void rosfrc::EncoderUpdater::update()
{
	encoder_msg.distance = m_encoder->GetDistance();
	encoder_msg.rate = m_encoder->GetRate();
	pub.publish(&encoder_msg);
}
rosfrc::Accelerometer::Accelerometer(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Accelerometer> accelerometer) :
	m_accelerometer(accelerometer),
	pub(topic, &accel_msg)
{
	nh.advertise(pub);
}
rosfrc::Accelerometer::Accelerometer(ros::NodeHandle& nh, const char* topic, frc::Accelerometer* accelerometer) :
	rosfrc::Accelerometer(nh, topic, std::shared_ptr<frc::Accelerometer>(accelerometer))
{

}
void rosfrc::Accelerometer::update()
{
	accel_msg.x = m_accelerometer->GetX();
	accel_msg.y = m_accelerometer->GetY();
	accel_msg.z = m_accelerometer->GetZ();
	pub.publish(&accel_msg);
}

rosfrc::GyroUpdater::GyroUpdater(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Gyro> gyro) :
	m_gyro(gyro),
	pub(topic, &gyro_msg)
{
	nh.advertise(pub);
}
rosfrc::GyroUpdater::GyroUpdater(ros::NodeHandle& nh, const char* topic, frc::Gyro* gyro) :
	GyroUpdater(nh, topic, std::shared_ptr<frc::Gyro>(gyro))
{

}
void rosfrc::GyroUpdater::update()
{
	gyro_msg.angle = m_gyro->GetAngle();
	gyro_msg.rate = m_gyro->GetRate();
	pub.publish(&gyro_msg);
}
