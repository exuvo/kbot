kbotpi
======

Kartobot RaspberryPi

TODO description of project

**Communication**
* internal
  * with lpc1768 (rest of the robot) using serial port (UART).
  * 1 pin connected to lpc1768 boot-request for programming.
* external
  * short range: usb wifi for normal control over tcp/udp.
  * long range: xbee connected to lpc1768.
  local ssh console ncurses gui

**Program environment**
* language: c++ (others ex python allowed for testing)
* make utility: cmake
* version control: git (@github)


**Objectives**
* Read data from serial port to lpc1768
  * Read data from xbee sent thru lpc1768
  * sensors: sonar, motor status, battery level
* Read data from network (wifi)
  * commands from computer/phone
  * send status and video
* Create map from sonar data
  * avoid hitting walls
* Detect things with camera
  * library openCV
  * objects to detect: doors

**Structure**
* lpc1768 communication: get sensor values, send motor commands
  * Serial port reader
* Communication: Maintain list of connected clients.
  * Parser: Understand commands. aware where a request came from and respond correctly.
  * Connection sources:
    * wifi tcp ip
    * xbee
* Map
  * Mapping: Take sonar pings from lpc1768 and place on map as a arc
  * Navigation: navigate around things
  * Drift correction: try to correct for sensor drift over time
* Camera
  * Get: Take images
  * Detection: Detect objects/lines/patterns
  * Memory: Remember objects from last frames?
