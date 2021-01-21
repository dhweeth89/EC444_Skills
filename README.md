# EC444_Skills
These quests demonstrate various projects I have worked on in a Smart and Connected Systems (IoT) course.

Quest 0 is empty so disregard.

Quest 1 is a fish feeder, which uses RTOS functionality to display a timer and then run a servo motor periodically.

Quest 2 is a tactile internet project, which incorporates Node.js and CSV data to graphically display sensor (Ultrasonic, Infrared, Thermistor
data in real time on a webpage.

Quest 3 is a remote hurricane box project, which uses thermistors to track temperature, LEDs controlled via webpage, and accelerometers
to track box "vibrations". It also has Raspberry Pi camera streaming capabilities, with the ability to watch via webpage.

Quest 4 is a e-voting project, where multiple IR receivers and transmitters and multiple microcontrollers are used to host an election
between various colored LEDs. Elections can be held remotely around the world, and the voting results are sent to a common server, where
the data is processed and a tally for candidates is kept. This uses bully election algorithms.

Quest 5 is an autonomous toy car project that uses sensors to detect speed and distance from walls/obstacles. This data is used to
dynamically course-correct the vehicle.

Quest 6 is a self-conceptualized project that simulates a factory. One goal is to make a scanner for a hypothetical conveyor belt, using
sensors to determine if an object is correctly in place for processing, with confirmation indicated by both LED and server-side logging.
The factory also has a fan to regulate temperature, which can be triggered remotely via webpage button (highest priority). It also runs
if thermistors detect unacceptable temperatures (next priority).  Finally, it also runs periodically, according to a timer displayed on
a 14-segment display. The fan also has an acclerometer connected to it, which sends an alert to the remote webpage if it detects abnormal 
movement, only disappearing once the hardware is fixed. Finally, the factory also is watched via camera, which streams to the webpage.
