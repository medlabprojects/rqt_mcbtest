#ifndef rqt_mcbtest__mcbtest_H
#define rqt_mcbtest__mcbtest_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_mcb_gui.h>
#include "mcbros.h"
#include <QWidget>
#include <QVector>
#include <vector>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <qwt_counter.h>
#include "ros/ros.h"
#include "medlab_motor_control_board/McbEncoders.h"

namespace rqt_mcbtest {

class McbTest
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:

  McbTest();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();


protected slots:

  void connectNode(void);
  void connectionEstablished(void);
  void connectionLost(void);
  void controlStateChanged(bool controlState);
  void slot_motorStateChanged(int motor);
  void publishEnableRos(bool enable);
  void enableAllMotors(void);
  void disableAllMotors(void);
  void slot_checkBox_motorEnable(int motor);
  void zeroCurrentPosition(int motor);
  void publishGetStatus(void);
  void newDesiredPosition(int motor);
  void updatePositionLabels(medlab_motor_control_board::McbEncoderCurrent positions);
  void slot_newStatus(void); // connected to McbRos::newStatus() signal
  void slot_limitSwitchEvent(int motor, bool state);
  void setGainsDialog(int motor);


private:

  Ui::Mcb_Ui ui_;
  QVector< QLabel* > label_positionCurrent_;
  QVector< QwtCounter* > counter_positionDesired_;
  QVector< QCheckBox* > checkBox_motorEnable_;
  QVector< QPushButton* > button_zeroEncoder_;
  QVector< QPushButton* > button_pid_;
  QVector< QLabel* > label_limit_;
  QVector< QLabel* > label_effort_;
  QWidget* widget_;

  McbRos* motorBoard_;

  ros::Timer statusTimer_;
  double statusTimerInterval_;
  ros::Publisher pubStatus_;
  void callbackStatusTimer(const ros::TimerEvent &e);
  int watchdog_; // counts the number of status requests since the last response
  const int watchdogLimit_; // number of status requests without response before emitting connectionLost()

  void publishEnableAllMotors(bool enable);
  void initUiNames(void);
  const uint8_t maxMotors_;
  int numMotorsDetected_;
};
} // namespace
#endif // my_namespace__my_plugin_H
