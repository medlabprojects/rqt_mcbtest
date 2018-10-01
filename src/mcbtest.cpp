#include <mcbtest.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QString>
#include <QSignalMapper>
#include <QVector>
#include <vector>
#include <string>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "medlab_motor_control_board/McbEncoders.h"


namespace rqt_mcbtest {

McbTest::McbTest()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , maxMotors_(6)
  , statusTimerInterval_(0.05)
  , watchdogLimit_(100)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("McbTest");
}

void McbTest::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // assemble ui elements into QVectors for easier access
  initUiNames();

  // connect signals/slots
  connect(ui_.button_connectNode, SIGNAL(pressed()), this, SLOT(connectNode()));
  connect(ui_.button_getStatus, SIGNAL(pressed()), this, SLOT(publishGetStatus()));

  // map the counter_positionDesired signals to a single slot
  QSignalMapper *signalMapperCounter = new QSignalMapper(this);
  connect(signalMapperCounter, SIGNAL(mapped(int)), this, SLOT(newDesiredPosition(int)));
  for(int ii=0; ii<maxMotors_; ii++){
    signalMapperCounter->setMapping(counter_positionDesired_.at(ii), ii);
    connect(counter_positionDesired_.at(ii), SIGNAL(valueChanged(double)), signalMapperCounter, SLOT(map()));
  }

  // map the checkBox_motorEnable signals to a single slot
  QSignalMapper *signalMapperCheckboxMotorEnable_ = new QSignalMapper(this);
  connect(signalMapperCheckboxMotorEnable_, SIGNAL(mapped(int)), this, SLOT(slot_checkBox_motorEnable(int)));
  for(int ii=0; ii<maxMotors_; ii++){
    signalMapperCheckboxMotorEnable_->setMapping(checkBox_motorEnable_.at(ii), ii);
    connect(checkBox_motorEnable_.at(ii), SIGNAL(clicked()), signalMapperCheckboxMotorEnable_, SLOT(map()));
  }

  // map the button_zeroEncoder signals to a single slot
  QSignalMapper *signalMapperZeroEncoder = new QSignalMapper(this);
  connect(signalMapperZeroEncoder, SIGNAL(mapped(int)), this, SLOT(zeroCurrentPosition(int)));
  for(int ii=0; ii<maxMotors_; ii++){
    signalMapperZeroEncoder->setMapping(button_zeroEncoder_.at(ii), ii);
    connect(button_zeroEncoder_.at(ii), SIGNAL(pressed()), signalMapperZeroEncoder, SLOT(map()));
  }
}

void McbTest::publishEnableRos(bool enable)
{
  // send message to enable ROS control
  motorBoards_.at(0)->enableRosControl(enable);
}

void McbTest::enableAllMotors()
{
  publishEnableAllMotors(true);
}

void McbTest::disableAllMotors()
{
  publishEnableAllMotors(false);
}

void McbTest::slot_checkBox_motorEnable(int motor)
{
  bool enable = checkBox_motorEnable_.at(motor)->isChecked();
  motorBoards_.at(0)->enableMotor(motor, enable);
}

void McbTest::zeroCurrentPosition(int motor)
{
  if(motorBoards_.at(0)->isRosControlEnabled()){
    bool motorWasEnabled = motorBoards_.at(0)->isMotorEnabled(motor);
    motorBoards_.at(0)->zeroCurrentPosition(motor);
    counter_positionDesired_.at(motor)->setValue(0.0);
//    ROS_INFO("%d",motorWasEnabled);

    // motors are automatically disabled after zeroing, so we must update status and then re-enable
//    publishGetStatus();
    if(motorWasEnabled){
      ROS_INFO("enable motor");
      motorBoards_.at(0)->enableMotor(motor, true);
    }
  }
}

void McbTest::publishEnableAllMotors(bool enable)
{
  motorBoards_.at(0)->enableAllMotors(enable);
}

void McbTest::newDesiredPosition(int motor)
{
  motorBoards_.at(0)->setDesiredPosition(motor, (int32_t)counter_positionDesired_.at(motor)->value());
}

void McbTest::updatePositionLabels(medlab_motor_control_board::McbEncoderCurrent positions)
{
  for(int ii=0; ii<maxMotors_; ii++){
    label_positionCurrent_.at(ii)->setText(QString::number(positions.measured[ii]));
  }
}

void McbTest::slot_newStatus()
{
  // reset watchdog
  watchdog_ = 0;

  // update control effort labels
  QVector<float> efforts = motorBoards_.at(0)->getEfforts();
  for(int ii=0; ii<maxMotors_; ii++){
    label_effort_.at(ii)->setText(QString::number(efforts.at(ii),'f',6));
  }

  // update state label
  ui_.label_mcbState->setText((motorBoards_.at(0)->isRosControlEnabled() ? "Control" : "Idle"));

  // update motor states
  for(int ii=0; ii<maxMotors_; ii++){
    checkBox_motorEnable_.at(ii)->setChecked(motorBoards_.at(0)->isMotorEnabled(ii));
  }
}

void McbTest::initUiNames()
{
  label_positionCurrent_.reserve(maxMotors_);
  label_positionCurrent_.push_back(ui_.label_positionCurrent0);
  label_positionCurrent_.push_back(ui_.label_positionCurrent1);
  label_positionCurrent_.push_back(ui_.label_positionCurrent2);
  label_positionCurrent_.push_back(ui_.label_positionCurrent3);
  label_positionCurrent_.push_back(ui_.label_positionCurrent4);
  label_positionCurrent_.push_back(ui_.label_positionCurrent5);

  counter_positionDesired_.reserve(maxMotors_);
  counter_positionDesired_.push_back(ui_.counter_positionDesired0);
  counter_positionDesired_.push_back(ui_.counter_positionDesired1);
  counter_positionDesired_.push_back(ui_.counter_positionDesired2);
  counter_positionDesired_.push_back(ui_.counter_positionDesired3);
  counter_positionDesired_.push_back(ui_.counter_positionDesired4);
  counter_positionDesired_.push_back(ui_.counter_positionDesired5);

  checkBox_motorEnable_.reserve(maxMotors_);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable0);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable1);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable2);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable3);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable4);
  checkBox_motorEnable_.push_back(ui_.checkBox_motorEnable5);

  button_zeroEncoder_.reserve(maxMotors_);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder0);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder1);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder2);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder3);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder4);
  button_zeroEncoder_.push_back(ui_.button_zeroEncoder5);

  label_limit_.reserve(maxMotors_);
  label_limit_.push_back(ui_.label_limit0);
  label_limit_.push_back(ui_.label_limit1);
  label_limit_.push_back(ui_.label_limit2);
  label_limit_.push_back(ui_.label_limit3);
  label_limit_.push_back(ui_.label_limit4);
  label_limit_.push_back(ui_.label_limit5);

  label_effort_.reserve(maxMotors_);
  label_effort_.push_back(ui_.label_effort0);
  label_effort_.push_back(ui_.label_effort1);
  label_effort_.push_back(ui_.label_effort2);
  label_effort_.push_back(ui_.label_effort3);
  label_effort_.push_back(ui_.label_effort4);
  label_effort_.push_back(ui_.label_effort5);
}

void McbTest::shutdownPlugin()
{
  publishEnableRos(false);
  motorBoards_.remove(0);
  pubStatus_.shutdown();
}

void McbTest::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void McbTest::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void McbTest::connectNode()
{
  ui_.label_mcbState->setText("CONNECTING...");

  // initialize MCB1
  motorBoards_.push_back(new McbRos);
  std::string nodeName = ui_.lineEdit_nodeName->text().toStdString();
  motorBoards_.at(0)->init(nodeName);

  // connect McbRos signals
  connect(motorBoards_.at(0), SIGNAL(controlStateChanged(bool)), this, SLOT(controlStateChanged(bool)));
  connect(motorBoards_.at(0), SIGNAL(connectionEstablished()), this, SLOT(connectionEstablished()));
  connect(motorBoards_.at(0), SIGNAL(connectionLost()), this, SLOT(connectionLost()));
  connect(motorBoards_.at(0), SIGNAL(newPositions(medlab_motor_control_board::McbEncoderCurrent)), this, SLOT(updatePositionLabels(medlab_motor_control_board::McbEncoderCurrent)));
  connect(motorBoards_.at(0), SIGNAL(newStatus()), this, SLOT(slot_newStatus()));

  // start timer to regularly request status
  std::string topicGetStatus = "/" + nodeName + "/get_status";
  pubStatus_ = getNodeHandle().advertise<std_msgs::Empty>(topicGetStatus.c_str(),1);
  watchdog_ = 0;
  statusTimer_ = getNodeHandle().createTimer(ros::Duration(statusTimerInterval_), &McbTest::callbackStatusTimer, this);

  // disconnect button
  ui_.button_connectNode->setCheckable(false);
  ui_.button_connectNode->disconnect();
}

void McbTest::connectionEstablished()
{
  ui_.label_mcbState->setText("CONNECTED");

  // change connect button to disconnect
  ui_.button_connectNode->setText("Disconnect");
  ui_.button_connectNode->disconnect();
  connect(ui_.button_connectNode, SIGNAL(pressed()), this, SLOT(connectionLost()));

  // display IP and MAC addresses
  ui_.label_ip->setText(motorBoards_.at(0)->getIp());
  ui_.label_mac->setText(motorBoards_.at(0)->getMac());

  // setup button_enableRosControl and ensure we are in the Idle state
  ui_.button_enableRosControl->setChecked(false);
  connect(ui_.button_enableRosControl, SIGNAL(toggled(bool)), this, SLOT(publishEnableRos(bool)));
  publishEnableRos(false);
}

void McbTest::connectionLost()
{
  // stop statusTimer_
  statusTimer_.stop();

  // set UI elements as if we are in the Idle state
  publishEnableRos(false);
  controlStateChanged(false);

  // remove motor board node
  motorBoards_.remove(0);

  // change disconnect button to connect
  ui_.button_connectNode->setText("Connect");
  ui_.button_connectNode->disconnect();
  connect(ui_.button_connectNode, SIGNAL(pressed()), this, SLOT(connectNode()));

  // update labels
  ui_.label_mcbState->setText("DISCONNECTED");
  ui_.label_ip->setText("X.X.X.X");
  ui_.label_mac->setText("X:X:X:X:X:X");
}

void McbTest::slot_motorStateChanged(int motor)
{
  // check current motor state
  bool enabled = motorBoards_.at(0)->isMotorEnabled(motor);

  // effectively enables/disables counter
  counter_positionDesired_.at(motor)->setSingleStep((enabled ? 1.0 : 0.0));
}

void McbTest::controlStateChanged(bool controlState)
{
  if(controlState){
    // allow motors to be individually enabled
    numMotorsDetected_ = motorBoards_.at(0)->getNumMotors();
    if(numMotorsDetected_ > -1){
      for(int ii=0; ii<numMotorsDetected_; ii++){
        checkBox_motorEnable_.at(ii)->setCheckable(true);
      }
    }
    connect(motorBoards_.at(0), SIGNAL(motorStateChanged(int)), this, SLOT(slot_motorStateChanged(int)));
    ui_.button_enableRosControl->setText("Disable ROS Control");

    // setup buttons
    ui_.button_enableAllMotors->setChecked(false);
    ui_.button_disableAllMotors->setChecked(false);
    ui_.button_enableAllMotors->setCheckable(false);
    ui_.button_disableAllMotors->setCheckable(false);
    connect(ui_.button_enableAllMotors, SIGNAL(pressed()), this, SLOT(enableAllMotors()));
    connect(ui_.button_disableAllMotors, SIGNAL(pressed()), this, SLOT(disableAllMotors()));
  }
  else{
    // update button text
    ui_.button_enableRosControl->setText("Enable ROS Control");

    // disable motor checkboxes
    for(int ii=0; ii<maxMotors_; ii++){
      checkBox_motorEnable_.at(ii)->setChecked(false);
      checkBox_motorEnable_.at(ii)->setCheckable(false);
    }

    // disable buttons
    ui_.button_enableAllMotors->setCheckable(true);
    ui_.button_enableAllMotors->setChecked(true);
    ui_.button_disableAllMotors->setCheckable(true);
    ui_.button_disableAllMotors->setChecked(true);
    ui_.button_enableAllMotors->disconnect();
    ui_.button_disableAllMotors->disconnect();
  }
}

void McbTest::callbackStatusTimer(const ros::TimerEvent& e){
  // request status update
//  publishGetStatus();
  std_msgs::Empty msg;
  pubStatus_.publish(msg);

  // check for timeout
  if(watchdog_ > watchdogLimit_){
    connectionLost();
  }
  else{
    // increment watchdog
    watchdog_++;
  }

}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_mcbtest, McbTest, rqt_mcbtest::McbTest, rqt_gui_cpp::Plugin)
