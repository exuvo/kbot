kbotpi
======

This is the control program for a tracked robot called Kartobot. 
It uses active sonar and a camera to navigate its surroundings. Listens to high-level remote control from a pc or mobile phone. The goal is for it to be able to survive handling the controls to a 5 year old. It should activly avoid hitting walls even if it is receiving commands to do so.

[Build log](http://elektronikforumet.com/forum/viewtopic.php?f=3&t=66441) (requires registration to view pictures there, [pictures only here](http://exuvo.se/gallery/usergallery/exuvo/gallery/gallery/kbot)).

**Communication**
* internal
  * with [Zircon 4](https://oshpark.com/shared_projects/L3JRpNqU) (robot mainboard with a STM32F437ZGT6) using serial port over USB.
* external
  * short range: usb wifi for normal control over tcp/udp.
  * long range: xbee connected to Zircon 4.
  * ncurses gui over ssh for settings and system status

**Program environment**
* language: c++
* libraries: ROS
* make utility: cmake
* version control: git (@github)
* Runs on a [Parallella](http://www.parallella.org/board/)
* Vision code will probably run on a Raspberry Pi
* Code style is similar to [standard ROS](http://www.ros.org/wiki/CppStyleGuide)

**Objectives**
* Read data from serial port connected to Zircon 4
  * Read data forwarded from Zircon 4s XBee
  * sensors: sonar, motor status, battery level
* Read data from network (wifi)
  * commands from computer/phone
  * send status and video
* Create map from sonar data
  * avoid hitting walls
* Detect things with camera
  * library openCV?
  * objects to detect: doors, colored items
  * utilize Epiphany coprocessor

**Structure**
* Zircon 4 communication: get sensor values, send motor commands
  * Serial port reader (kbot\_bridge)
* Network: Maintain list of connected clients
  * Parser: Understand commands. aware where a request came from and respond correctly
  * Connection sources: wifi tcp, xbee
* Map
  * Mapping: Take sonar pings from Zircon 4 (kbot\_bridge) and place on map as an arc (kbot\_mapper)
  * Navigation: navigate around things
  * Drift correction: try to correct for sensor drift over time
* Vision
  * Get: Take images
  * Detection: Detect objects/lines/patterns
  * Memory: Remember objects from last frames?

**Libraries**
* ROS
    * MoveIt
    * OctoMap
    * [serial](https://github.com/wjwwood/serial)
    * tf2
* External
    * [gflags](https://code.google.com/p/gflags/)
    * [ezesdk](http://www.users.on.net/~notzed/software/ezesdk.html)
