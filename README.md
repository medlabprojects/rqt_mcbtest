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
 2. Install QT5
  - First, check the current QT version by running the command
    ```
    qmake --version
    ```
    If you have QT5-XXX, you don't need to do the following.
    If you dont have QT5, install it by running the command:
    ```
    sudo apt-get install qt5-default
    ```
    Once we made sure that we have QT5, we also need to install the SVG libraries which is needed for compiling the Qwt later. Run the command:
    ```
    sudo apt-get install libqt5svg5-dev
    ```
 3. Download and compile Qwt (more info and full instructions: [https://qwt.sourceforge.io/qwtinstall.html](https://qwt.sourceforge.io/qwtinstall.html)
   - Download the .zip of the latest version from [https://sourceforge.net/projects/qwt/files/qwt/](https://sourceforge.net/projects/qwt/files/qwt/)
   - Extract somewhere (location does not matter)
   - Open a terminal and navigate to that directory
   - Build and install
   ```
   qmake qwt.pro
   make
   sudo make install
   ```
  4. Setting the Qwt library location in the CMakeList.txt:
    - Open a terminal, and type
    ```
    ls /usr/local/
    ```
    - to find out what is the path of the Qwt installed. The file with the "qwt-XXX" name is our installed Qwt location. We need to update that in our CMakeList.  
    - Go to your catkin workspace and open the CMakeList.txt for the rqt_mcbtest package. Go the line with "QWT_DIR" (close the top file), and change the existing line below from "/usr/local/qwt-6.1.4-qt-5.5.1" to your "/usr/local/qwt-XXX". Save the CMakeList.txt and close.
    - Do a catkin_make to compile the package.
    
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
