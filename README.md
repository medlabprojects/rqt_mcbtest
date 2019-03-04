# rqt_mcbtest

**rqt plugin that provides a GUI for controlling the MED Lab Motor Control Boards**

## Instructions (tested on Ubuntu 16.04 with ROS Kinetic)
### Prerequisites
  - At least one motor board running the [MCB_ROS](https://github.com/tlbruns/MCB_ROS) firmware
  - The [medlab_motor_control_board](https://github.com/vanderbiltmedlab/medlab_motor_control_board) package
  
### Setup
1. Download and extract repository to your catkin workspace
   - Using GUI:
      - Click the green 'Clone or Download' button above
      - Click 'Download ZIP'
      - Open with Archive Manager
      - Click 'Extract' and choose your desired location
   - Using Terminal/Command Line:
      - Download ZIP
      ```
      wget -O ./rqt_mcbtest.zip "https://github.com/vanderbiltmedlab/rqt_mcbtest/zipball/master"
      ```
      - Extract to your desired location
      ```
      unzip rqt_mcbtest.zip -d /path/to/catkin/workspace/src
      ```
 2. Download and compile Qwt (more info and full instructions: [https://qwt.sourceforge.io/qwtinstall.html](https://qwt.sourceforge.io/qwtinstall.html)
   - Download the .zip of the latest version from [https://sourceforge.net/projects/qwt/files/qwt/](https://sourceforge.net/projects/qwt/files/qwt/)
   - Extract somewhere (location does not matter)
   - Open a terminal and navigate to that directory
   - Build and install
   ```
   /usr/local/Qt-5.0.1/bin/qmake qwt.pro
   make
   sudo make install
   ```
### How to Use
1. Rebuild your catkin workspace (e.g. 'catkin_make')
2. Start rosserial_server
  ```
  roslaunch rosserial_server socket.launch 
  ```
3. Start rqt_mcbtest
  ```
  rqt --standalone rqt_mcbtest
  ```
4. Enter the namespace of your motor board and click 'Connect'
5. Click 'Enable ROS Control'
6. Enable motors individually using checkboxes next to each or all together with 'Enable All Motors'
7. Jog positions using the arrows under 'Desired Positions'
