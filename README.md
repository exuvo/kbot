kbotpi
======

This is the control program for a tracked robot called Kartobot. 
It uses active sonar and a camera to navigate its surroundings.
The goal is for it to be able to survive independently while also 
supporting high-level remote control from a pc or mobile phone.

**Communication**
* internal
  * with lpc1768 (rest of the robot) using serial port (UART).
  * 1 pin connected to lpc1768 boot-request for programming.
* external
  * short range: usb wifi for normal control over tcp/udp.
  * long range: xbee connected to lpc1768.
  * local ssh console ncurses gui

**Program environment**
* language: c++
* libraries: ROS
* make utility: cmake
* version control: git (@github)
* Runs on a Raspberry Pi Model B with 256MB RAM, 800Mhz
* Will run on a http://www.parallella.org/board/ in the far future.
* Code style http://www.ros.org/wiki/CppStyleGuide

**Objectives**
* Read data from serial port connected to lpc1768
  * Read data from xbee sent thru lpc1768
  * sensors: sonar, motor status, battery level
* Read data from network (wifi)
  * commands from computer/phone
  * send status and video
* Create map from sonar data
  * avoid hitting walls
* Detect things with camera
  * library openCV
  * objects to detect: doors, colored items
  * utilize Epiphany coprocessor http://www.parallella.org/board/

**Structure**
* lpc1768 communication: get sensor values, send motor commands
  * Serial port reader (kbot\_bridge)
* Network: Maintain list of connected clients.
  * Parser: Understand commands. aware where a request came from and respond correctly.
  * Connection sources: wifi tcp, xbee
* Map
  * Mapping: Take sonar pings from lpc1768 (kbot\_bridge) and place on map as a arc (kbot\_mapper)
  * Navigation: navigate around things
  * Drift correction: try to correct for sensor drift over time
* Vision
  * Get: Take images
  * Detection: Detect objects/lines/patterns
  * Memory: Remember objects from last frames?

**Libraries**
* ROS: MoveIt
* ROS: OctoMap
* ROS: serial
* ROS: tf2
* GOOGLE: gflags
