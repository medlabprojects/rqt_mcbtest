#include <rqt_mcbtest/mcbtest.h>
#include <rqt_mcbtest/gainsdialog.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QString>
#include <QVector>
#include <memory>
#include <string>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "medlab_motor_control_board/McbEncoders.h"

using namespace medlab_motor_control_board;

namespace rqt_mcbtest {

McbTest::McbTest()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , maxMotors_(6)
  , numMotorsDetected_(-1)

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

  // connect gui buttons
  connect(ui_.button_connectNode, SIGNAL(pressed()),
          this, SLOT(connectNode()));

  // map the button_zeroEncoder signals to a single slot
  for(int ii=0; ii<button_zeroEncoder_.size(); ii++){
    connect(button_zeroEncoder_[ii], &QAbstractButton::pressed,
            this, [this, ii](){zeroCurrentPosition(ii);});
  }
}

void McbTest::publishEnableRos(bool enable)
{
  // send message to enable ROS control
  motorBoard_->enableRosControl(enable);
}

void McbTest::slot_checkBox_motorEnable(int motor)
{
  // see if box was checked or unchecked, then issue command
  bool enable = checkBox_motorEnable_.at(motor)->isChecked();
  motorBoard_->enableMotor(motor, enable);

  // clear a potential "Triggered" label if enabling the motor
  if(enable){
    label_limit_.at(motor)->clear();
  }
}

void McbTest::zeroCurrentPosition(int motor)
{
  if(motorBoard_->isRosControlEnabled()){
    bool motorWasEnabled = motorBoard_->isMotorEnabled(motor);
    motorBoard_->zeroCurrentPosition(motor);
    counter_positionDesired_.at(motor)->setValue(0.0);

    // motors are automatically disabled after zeroing, so we must re-enable
    if(motorWasEnabled){
      ROS_INFO("enable motor"); // BUG: removing this line causes motor to not re-enable when
                                // the desired position is set to 0. Messages sent too fast?
      motorBoard_->enableMotor(motor, true);
    }
  }
}

void McbTest::zeroCurrentPositions()
{
  for(int ii=0; ii<numMotorsDetected_; ii++){ // inefficient, but reliable and called rarely
    zeroCurrentPosition(ii);
  }
}

void McbTest::publishEnableAllMotors(bool enable)
{
  // set counters to the current positions (prevents accidentally commanding large steps)
  auto curPositions = motorBoard_->currentPositions();
  for(int motor=0; motor<counter_positionDesired_.size(); motor++){
    counter_positionDesired_.at(motor)->setValue(curPositions.measured[motor]);
  }

  // publish enable message
  motorBoard_->enableAllMotors(enable);
}

void McbTest::newDesiredPosition(int motor)
{
  motorBoard_->setDesiredPosition(motor, static_cast<int32_t>(counter_positionDesired_.at(motor)->value()));
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
//  watchdog_ = 0;

  // update control effort labels
  QVector<float> efforts = motorBoard_->getEfforts();
  for(int ii=0; ii<maxMotors_; ii++){
    label_effort_.at(ii)->setText(QString::number(efforts.at(ii),'f',6));
  }

  // update state label
  ui_.label_mcbState->setText((motorBoard_->isRosControlEnabled() ? "Control" : "Idle"));

  // update motor states
  for(int ii=0; ii<maxMotors_; ii++){
    checkBox_motorEnable_.at(ii)->setChecked(motorBoard_->isMotorEnabled(ii));
  }
}

void McbTest::slot_limitSwitchEvent(int motor, bool state)
{
  // check if the E-Stop was triggered
  if(motor == 6){
    for(auto& label_limit : label_limit_){
      state ? label_limit->setText("E-STOP!")
            : label_limit->clear();
    }
  }
  else if(motor < 6){
    label_limit_.at(motor)->setText("Triggered");
  }
}

void McbTest::setGainsDialog(int motor)
{
  // get the current gain values
  double p = motorBoard_->getP(motor);
  double i = motorBoard_->getI(motor);
  double d = motorBoard_->getD(motor);

  // create popup dialog
  GainsDialog *gainsWindow = new GainsDialog(motor, p, i, d);
  gainsWindow->setModal(true); // lock main gui until dialog closed

  // connect newGains signal to setGains slot
  connect(gainsWindow, SIGNAL(newGains(quint8,double,double,double)),
          motorBoard_.get(), SLOT(setGains(quint8,double,double,double)));

  // open dialog window
  gainsWindow->show();
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

  button_pid_.reserve(maxMotors_);
  button_pid_.push_back(ui_.button_pid0);
  button_pid_.push_back(ui_.button_pid1);
  button_pid_.push_back(ui_.button_pid2);
  button_pid_.push_back(ui_.button_pid3);
  button_pid_.push_back(ui_.button_pid4);
  button_pid_.push_back(ui_.button_pid5);

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
}

void McbTest::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)

  // save node name for next session
  instance_settings.setValue("nodeName", ui_.lineEdit_nodeName->text());
}

void McbTest::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)

  // restore previous node name
  if(instance_settings.contains("nodeName")){
    QString nodeName = instance_settings.value("nodeName", "").toString();
    ui_.lineEdit_nodeName->clear();
    ui_.lineEdit_nodeName->insert(nodeName);
  }

}

void McbTest::connectNode()
{
  ui_.label_mcbState->setText("CONNECTING...");

  // initialize MCB1
  motorBoard_ = std::make_unique<medlab_motor_control_board::McbRos>();
  std::string nodeName = ui_.lineEdit_nodeName->text().toStdString();
  motorBoard_->init(nodeName);

  // connect McbRos signals/slots
  connect(motorBoard_.get(), SIGNAL(controlStateChanged(bool)),
          this,              SLOT(controlStateChanged(bool)));

  connect(motorBoard_.get(), SIGNAL(connectionEstablished()),
          this,              SLOT(connectionEstablished()));

  connect(motorBoard_.get(), SIGNAL(connectionLost()),
          this,              SLOT(connectionLost()));

  connect(motorBoard_.get(), SIGNAL(newPositions(medlab_motor_control_board::McbEncoderCurrent)),
          this,              SLOT(updatePositionLabels(medlab_motor_control_board::McbEncoderCurrent)));

  connect(motorBoard_.get(), SIGNAL(newStatus()),
          this,              SLOT(slot_newStatus()));

  connect(motorBoard_.get(), SIGNAL(limitSwitchEvent(int,bool)),
          this,              SLOT(slot_limitSwitchEvent(int,bool)));

  for(int ii=0; ii<counter_positionDesired_.size(); ii++){
    // map the counter_positionDesired signals to a single slot
    connect(counter_positionDesired_[ii], &QwtCounter::valueChanged, this, [this, ii](double newValue){
      motorBoard_->setDesiredPosition(ii, static_cast<int32_t>(newValue));
    });
  }

  for(int ii=0; ii<checkBox_motorEnable_.size(); ii++){
    // map the checkBox_motorEnable signals to a single slot
    connect(checkBox_motorEnable_[ii], &QAbstractButton::clicked, this, [this, ii](){
      slot_checkBox_motorEnable(ii);
    });
  }

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
  connect(ui_.button_connectNode, SIGNAL(pressed()),
          this, SLOT(connectionLost()));

  // display IP and MAC addresses
  ui_.label_ip->setText(motorBoard_->getIp());
  ui_.label_mac->setText(motorBoard_->getMac());

  // setup button_enableRosControl and ensure we are in the Idle state
  ui_.button_enableRosControl->setChecked(false);
  connect(ui_.button_enableRosControl, SIGNAL(toggled(bool)),
          this, SLOT(publishEnableRos(bool)));
  publishEnableRos(false);

  // connect buttons for setting PID gains
  for(int ii=0; ii<button_pid_.size(); ii++){
    connect(button_pid_[ii], &QAbstractButton::pressed, this, [this, ii](){
      setGainsDialog(ii);
    });
  }
}

void McbTest::connectionLost()
{
  // set UI elements as if we are in the Idle state
  publishEnableRos(false);
  controlStateChanged(false);

  // remove motor board node
  motorBoard_.reset();

  // change disconnect button to connect
  ui_.button_connectNode->setText("Connect");
  ui_.button_connectNode->disconnect();
  connect(ui_.button_connectNode, SIGNAL(pressed()),
          this, SLOT(connectNode()));

  // disable pid gains buttons
  for(int ii=0; ii<button_pid_.size(); ii++){
    button_pid_.at(ii)->disconnect();
  }

  // update labels
  ui_.label_mcbState->setText("DISCONNECTED");
  ui_.label_ip->setText("X.X.X.X");
  ui_.label_mac->setText("X:X:X:X:X:X");
}

void McbTest::slot_motorStateChanged(int motor)
{
  // check current motor state
  bool enabled = motorBoard_->isMotorEnabled(motor);

  // enable/disable counter (effectively; set step size to 0)
  counter_positionDesired_.at(motor)->setSingleStep((enabled ? 1.0 : 0.0));
}

void McbTest::controlStateChanged(bool controlState)
{
  if(controlState){
    // allow motors to be individually enabled
    numMotorsDetected_ = motorBoard_->getNumMotors();
    if(numMotorsDetected_ > -1){
      for(int ii=0; ii<numMotorsDetected_; ii++){
        checkBox_motorEnable_.at(ii)->setCheckable(true);
      }
    }
    connect(motorBoard_.get(), SIGNAL(motorStateChanged(int)),
            this, SLOT(slot_motorStateChanged(int)));
    ui_.button_enableRosControl->setText("Disable ROS Control");

    // setup buttons
    ui_.button_enableAllMotors->setChecked(false);
    ui_.button_disableAllMotors->setChecked(false);
    ui_.button_zeroAll->setChecked(false);
    ui_.button_enableAllMotors->setCheckable(false);
    ui_.button_disableAllMotors->setCheckable(false);
    ui_.button_zeroAll->setCheckable(false);
    connect(ui_.button_enableAllMotors, &QAbstractButton::pressed, this, [this](){motorBoard_->enableAllMotors(true);});
    connect(ui_.button_disableAllMotors, &QAbstractButton::pressed, this, [this](){motorBoard_->enableAllMotors(false);});
    connect(ui_.button_zeroAll, &QAbstractButton::pressed,
            motorBoard_.get(), &McbRos::zeroCurrentPositions);
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
