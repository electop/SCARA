 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 132
A Desktop SCARA Robot using Stepper Motors
Pranav Shevkar1, Sayali Bankar2, Akash Vanjare3, Pranita Shinde4, Vaibhav Redekar5,
Shubham Chidrewar6
----------------------------------------------------------------------
Abstract – The paper describes the development and testing of 3 degree of freedom (DOF) desktop SCARA that is capable of picking and placing objects with high speed and precision. Using Autodesk inventor, the initial modelling of SCARA robot is created.
The rotational inertias of the two links are calculated from the software and using parallel axis theorem.
Using the considered values of peak speed and acceleration, torques of stepper motors are calculated.
The inverse kinematics calculations are carried out in MATLAB software since the processing power of microcontroller is limited. 
MATLAB is so programmed to convert the angles into number of steps and sent to the microcontroller by serial communication.
To enable the robotic arm to operate at high speed the inter-step delays for stepper motors are calculated using Taylor's series to provide constant acceleration and deceleration.
A graphical user interface is designed using MATLAB. Finally, the electronic circuitry and mechanical parts are assembled, and the robot is tested for repeatability.
Key Words: SCARA, 3-DOF manipulator, Pick-n-place, MATLAB, Inverse kinematics, Stepper motor.

1. INTRODUCTION
The use of robots in industries is proliferating due to the necessity of automation, reducing worker fatigue and faster productivity.
Robots applications in industries is primarily in four fields viz. material handling, operations, assembly and inspection.
Apart from industries, robots are also deployed in home sector, health care, service sector, agriculture and farms, research and exploration.
The applications of robots are only limited by the need and imagination of the developer and the end user.
Robots have a potential to change our economy, health, living style, and the world we live in [1].
The SCARA acronym stands for Selective Compliance Articulated Robot Arm.
It is selective compliant since the arm is fully compliant in the horizontal plane while it is rigid in the vertical axis.
Also, the two-link arm layout imitates the human arm thus making it articulated.
One joint acts as a shoulder joint and the second as an elbow joint [2].
SCARA robots are the first choice of industries due to high speed and accuracy.
These robots are used generally for assembly, pick and place, sorting, soldering and painting.
This work presents the design and fabrication of desktop robotic arm that can do its programmed movements with speed, accuracy and precision.
The focus is to bring robots for day to day applications for household and educational use.
This SCARA robot is ideal for applications involving the manipulator to travel fixed coordinates.
The robotic arm can be operated in manual as well as auto mode.
The user can enter the sequence of operations into the HMI along with the desired speed and dwell time.
The MATLAB program calculates the angles and instructs the stepper motor to attain the required coordinates.
The vacuum pump and vacuum cup enable the robot to be used for pick and place operations.
It is necessary to do referencing of the robot every time it is started [3].
The focus is to attain accuracy at high speed keeping the cost as low as possible.
The methodology followed for development of the proposed SCARA robot is shown in fig – 1.
Fig – 1: Methodology

2. REQUIRED SPECIFICATIONS
In order to meet the requirements, SCARA robot is expected to have specifications as mentioned in table 1.
Considering the application and the workspace, a robotic arm with R-R-P (Revolute, Revolute, Prismatic)
configuration is selected. Fig – 2 shows the schematic diagram of the robotic arm. The end effector consists of a
vacuum cup which will enable to pick and place objects.
Design Specification
Conceptual Design
Assembly
Trials
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 133
Table -1: Required Specifications
Required specifications
Specification Value
Degrees of Freedom 3
Maximum payload 100 g
Maximum reach 300 mm
Peak planar speed 200 deg/sec
Peak vertical speed 50 mm/sec
Repeatability 0.05 mm
Fig -2: Schematic diagram of Robotic arm.

3. DESIGN
The proposed design of SCARA robot developed using
Autodesk Inventor is shown in fig - 3 and the implemented
design is shown in fig - 4. The robot comprises of two links
in a parallel axis joint layout which are capable of
movement in horizontal plane and a lead screw nut
arrangement for movement in the z-axis. To ensure
smooth movement in the vertical axis, four linear bearings
are used along with guide rods to support the platform
that supports the stepper motors. Aluminium extrusion
beam is used as a substructure for rigid support of vertical
axis stepper motor. Each of the links, as well as the lead
screw, is driven by a separate stepper motor. The shaft of
the second link is connected to the stepper 2 using belt
and pulley arrangement to reduce the load on the arm.
This enables selecting a motor with lower torque which
eventually reduces cost. A 40 teeth pulley is used at output
and 20 teeth at the motor shaft. This gives a speed ratio of
0.5 for link 2. The end effector is introduced in such a way
that it allows various user requirements [4]. The materials
are selected such that the rotational moment and the
weight of the assembly are minimized. Motor supports and
links are manufactured from acrylic material by laser
cutting which also helps to reduce cost [5].
Fig - 3: 3D Model of Robotic arm.
Fig - 4: Actual model of Robotic arm.
The workspace of the SCARA arm forms a cardioid shape
with outer diameter 600mm & inner diameter 100mm.
Angular reach is 240 º. When the link 1 reaches ±120º the
link 2 can be further extended to 120º.
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 134
Fig – 5: Working Envelope.
4. INVERSE KINEMATICS
The inverse kinematics determine the angles θ1 and θ2 of
joint 1 and 2 respectively to bring the end effector to the
desired position (PX,PY) [6]. Cartesian coordinates of the
desired end effector position are entered in MATLAB
which calculate the angles and convert to number of steps.
Fig – 6: Inverse Kinematics diagram.
If the Cartesian coordinates of desired end-effector
location are given by (PX,PY) and l1 and l2 are link lengths
which are 150 mm each, the shoulder angle θ1and elbow
angle θ2 is given by equation (1) and (2).
θ2 =
(






) (1)
θ1 = (
) (





 √


) (2)
The function (
) can be defined as:
 (
) =
{


























Consider an end effector position P(184,206)
Since both the links are 150 mm each,
l1 = l2 = 150
Substituting the values in above equation,
θ2 =
(

 )
θ2 = 0.801 rad = 45.943 deg
θ1 = ( )
(

 √
)
θ1 = 0.441 rad = 25.257 deg
The calculations are verified using Inventor software.
Fig - 7: Verification of inverse kinematics.
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 135
5. MOTOR SELECTION
Due to high accuracy and repeatability, stepper motors are
selected for the application. To ensure proper working,
steppers having torque greater than the joint torque of
each joint are to be selected. The lagrange-euler
formulation is used to calculate the joint torque [7]. The zaxis motor is selected on the basis to the torque required
to overcome gravity and torque to overcome inertia.
Rotational Inertia at Joint 1= 20.88x10-3 Nm2
Rotational Inertia at Joint 2= 4.36x10-3 Nm2
The peak angular velocity is considered to be 200 deg/sec.
and this angular velocity is achieved in 0.33sec
For Joint 1,
α= 10.577 rad/sec2
τ=0.22Nm
=2.2kg cm
For Joint 2,
α= 10.577 rad/sec2
τ=0.093Nm
=0.93 kg-cm
Hence, motor selected for Joint 1 is NEMA17-4.2kg cm &
for joint 2 is NEMA17-1.4kg cm.
Peak lead screw nut speed is considered to be 50mm/sec.
A lead screw with four start and 2mm pitch is selected to
minimize the stepper revolution.
Peak lead screw speed =

= 39.26rad/sec
Assume lead screw efficiency =86%
Polar moment of inertia of lead screw (JS) =38.8 x 107kgm2
JL= 1.134x10-6 kg m2
JS+JL=5.01 x10-6
For vertical axis
Acceleration is considered to be 0.15m/sec2
α =117.92rad/sec2
Torque to overcome gravity =


= 0.1N-m
Τj = 5.01x10−6x0.15
= 7.515x10−7
τa = 7.515x10−7 + 0.1 = 0.1N-m = 1kg cm
Hence a stepper motor of 1.4 kg-cm is selected for vertical
axis movement.
6. NUMBER OF STEPS PER DEGREE
Since the input to stepper motors is given as number of
steps instead of angle, it is necessary to convert angle into
number of steps. As the configuration of all the three
steppers is different from each other, number of steps per
degree is also different. Stepper 1 is directly coupled to
link 1, Stepper 2 is connected to link 2 via belt pulley
arrangement with reduction ratio of 0.5, while the stepper
3 is connected to the lead screw having 4 starts directly
using a flexible coupling. The stepper motor driver
DRV8825 supports six micro stepping modes viz. M0, M1,
M2, M3, M4 and M5 for full step, half step, step,
step, 1/16 step and 1/32 step respectively. To achieve
maximum possible accuracy, M5 mode is selected with
1/32 step.
For stepper motor 1 (Link1):
I revolution (360˚) = 200×32 = 6400 steps
1˚ = 6400/360 = 17.778 steps.
For stepper motor 2 (Link2):
Reduction ratio = 0.5
1 revolution (360˚) = 2×200×32 = 12800 steps
1˚ = 12800/360 = 35.556 steps.
For stepper motor 2 (Vertical axis):
Lead screw pitch = 2 mm
Number of starts = 4
Lead = 8 mm
Thus, 1 mm = 1/8 revolution = 6400/8 = 800 steps.
7. INTER-STEP DELAY
To make sure that the stepper motor operates smoothly it
is necessary to accelerate and decelerate the stepper
motor while starting and stopping.
It can be seen from fig-8 that a linear speed increase can
be obtained using constant acceleration or deceleration
[8]. 
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 136
Fig - 8: Acceleration, Speed and Position.
Stepper motor speed is controlled by the time delay δt
between each stepper pulse. It is essential to calculate
these delays to make the speed of the stepper motor to
follow the desired speed slope [6].
Fig - 9: Speed profile against stepper pulses
The first counter delay co is given as,
co =


√

 ̇
and the nth counter delay cn is given as
cn = co (√ - √ )
where,
Φ=

rad
ω=


Owing to the limited computational power of
microcontrollers, it is necessary to simplify the equations
since it may take some time to compute two square roots.
Hence, using Taylor series approximation,
√








 (



)
Therefore,



 (√ √ )
 (√ √ )






 (


)
 (




 (


))



The expression for counter delay can be approximated as:



The time required by the microcontroller for this
calculation is much less than the double square root, but
when n = 1, an error of 0.44 is introduced. To compensate
this error, co can be multiplied by 0.676.
5. ELECTRONICS
Since the objective of the research is to make robots
available for day to day use, an open source, easily
reprogrammable and is readily available microcontroller
is needed. Arduino UNO meets all the requirements. A CNC
shield is used along with three DRV8825 motor drivers to
drive the stepper motors individually. This CNC-shield can
be mounted directly over the Arduino board. The motor
driver enables to micro step the stepper motor down to
1/32-step [9]. This helps in increasing the accuracy. Three
limit switches are used for referencing the robotic arm.
This eliminates the need for rotary encoders. The Arduino
board is powered by 5V DC power supply while the
stepper motors are powered by 12V DC power supply. The
Arduino board is connected to a computer via serial
communication [10]. 
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 137
Fig - 9: Block diagram of SCARA robot
Fig - 10: Electronic Circuit of SCARA robot
6. SOFTWARE
The graphical user interface required to control the robot
is designed using MATLAB. Since the processing power of
microcontroller is limited, all the calculations are
performed using MATLAB software. It converts the
Cartesian coordinates (Px, Py) into link angles θ1 and θ2.
Further, these angles are converted into steps. These
number of steps are sent to the microcontroller using
serial communication. The robotic arm can be operated in
manual or auto mode. To use the manual mode, the user
enters the desired cartesian coordinates. After entering
the coordinates and clicking update, it will move the end
effector to the desired location. Vacuum pump can be
switched on and off using the provided buttons. In the
auto mode, the user is required to record the sequence of
operation by entering the coordinates, the dwell time, and
the speed as 0-100%. In auto mode, this recorded
sequence can be executed either in a loop or wait after
execution. Since rotary encoders are not used, it is
necessary to reference the robotic arm every time it is
started using the special function in the software.
Whenever the auto mode is interrupted or stopped, it
should be referenced again. A status bar at the bottom of
the GUI shows the current state of the robotic arm. Fig - 11
shows the graphical user interface that is used to
communicate with the microcontroller.
Fig - 11: Graphical User Interface
7. TRIALS
The precision of the robotic arm is tested with the help of
a dial gauge having least count of 10 microns. The gauge is
placed in such a way that the probe of dial gauge touches
the arm [11, 12]. Subsequently the gauge was preset to
zero. The setup is shown in fig 12.
Fig - 12: Setup for repeatability trials
After performing series of movements, the arm was
brought to the same coordinate and the dial gauge
readings was noted. Ten such trials were conducted, and
the average repeatability of 0.1 mm was observed. Fig – 13
a and b demonstrate a sample reading.
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 138
Fig – 13 (a): 0 micron Fig – 13 (b): 100 microns
7. CONCLUSION
A 3 DOF SCARA robot is designed and developed for
carrying out day to day applications like pick and place.
The robot is first modelled in Inventor to get the values of
weight and inertia for stepper motor selection. The robotic
arm is then assembled using off the shelf parts to reduce
the cost of the robot. The end effector attachment is so
designed so that it can be modified according to
requirements. Arduino UNO is used along with motor
drivers to control the robotic arm. The stepper motor
drivers enable the stepper motors to be micro stepped to
1/32-step to increase accuracy. A specially designed GUI
using MATLAB serves as a human machine interface and
also converts cartesian coordinates to SCARA angles. The
GUI can record sequence of operations and execute the
sequence as per the need of the user. An algorithm for
accelerating and decelerating stepper motors is
introduced to ensure smooth working of the robot even at
high speeds. Finally, the robot is tested for repeatability
and is found to be within 0.1 mm which is within
acceptable limits.
8. FUTURE SCOPE
Presented research uses coordinates entered by the user
as input and records the sequence. Further a camera can
be used to capture the real time environment and with the
help of image processing, the robotic arm can determine
the coordinates to pick and place objects [13].
Furthermore, various control modes such as joystick,
mouse, gesture, etc. can be used to control the robotic arm.
The presented arm uses limit switches for referencing.
Using rotary encoders will eliminate the need of limit
switches and provide good positional accuracy [14]. The
robotic arm is designed to accommodate different types of
end effectors. Such different end effectors can be
developed and tested on this SCARA robot. Attaching an
extruder to the end effector can enable this robotic arm to
3D print objects with the help of provided STL file. With
the help of a laser engraver, this SCARA robot can be used
for laser engraving on surfaces such as wood, acrylic,
cardboard etc. A dedicated microcontroller and software
can be developed to compute faster and thereby increase
the speed of the SCARA robot [15].
REFERENCES
1) R.K. Mittal, I. J. Nagrath, Robotics and Control, McGraw
Hill Publications, 2016.
2) S. K. Saha, Introduction to Robotics, New Delhi: Tata
McGraw Hill Publications, 2008.
3) A. Nagchaudhuri, S. Kuruganty, A. Shakur,
“Introduction of mechatronics concepts in a robotics
course using an industrial SCARA robot equipped with
a vision sensor”, Mechatronics, Vol. 12, 2002 pp. 183–
193.
4) Angeles J, Morozov A, Navarro O, “A novel manipulator
architecture for the production of SCARA motions”,
Proceedings of the 2000 IEEE lntemational
Conference on Robotics and Automation, San
Francisco, CA, April 2000.
5) R.I. Eugene, Mechanical Design of Robots, McGrawHill, 1988.
6) M. Taylan Das, L. Canan D ̈lger, “Mathematical
modelling, simulation and experimental verification of
a scara robot”, Simulation Modelling Practice and
Theory, Vol.13 Jan. 2005, pp. 257–271.
7) D. W. Jones, “Control of Stepper Motors”, sections
5.2.10, 10.8, 10.9 and 10.10 of the Handbook of Small
Electric Motors edited by W. H. Yeadon and A. W.
Yeadon, McGraw-Hill, 2001.
8) D. Austin, “Generate stepper-motor speed profiles in
real time”, article in Embedded Systems
Programming’ January 2005.
9) K. S. M. Sahari and Hong Weng Khor, “Design And
Development Of A 4–Dof Scara Robot For Educational
Purposes”, Article in JurnalTeknologi, Jan 2011.
10) Ashly Baby, Chinnu Augustine, Chinnu Thampi, Maria
George, Abhilash A P,Philip C Jose “Pick and Place
Robotic Arm Implementation Using Arduino” IOSR
Journal of Electrical and Electronics Engineering Vol
12(2), Mar. – Apr. 2017, pp. 38-41.
11) K. Phasale, P. Kumar, A. Raut, R. R. Singh, A. Nichat,
“Design, Manufacturing and Analysis of Robotic Arm
with SCARA Configuration” International Research
Journal of Engineering and Technology Vol. 05(04)
Apr. 2018, pp. 82-85.
 International Research Journal of Engineering and Technology (IRJET) e-ISSN: 2395-0056
 Volume: 06 Issue: 02 | Feb 2019 www.irjet.net p-ISSN: 2395-0072
© 2019, IRJET | Impact Factor value: 7.211 | ISO 9001:2008 Certified Journal | Page 139
12) A. Omodei, G. Legnani, R. Adamini, “Three
methodologies for the calibration of industrial
manipulators: experimental results on a SCARA
robot”, Journal of Robotic Systems, Vol. 17(6), 2000,
pp.291–307.
13) M.J. Er, M.T. Lim, H.S. Lim, “Real time hybrid adaptive
fuzzy control of a SCARA robot”, Microprocessors and
Microsystems, Vol. 25, 2001, pp. 369–378.
14) Vincent Duchaine., Boris, Mayer, St-Onge., Clement,
Gosselin. and Dalong Gao., “Stable and Intuitive
Control of an Intelligent Assist Device”, IEEE
Transactions on haptics, 2012, Vol. 5(2), pp.148 – 159.
15) K.S. Hong, K.H. Choi, J.G. Kim, S. Lee, “A PC-based open
robot control system”, PC-ORC, Robotics and
Computer Integrated Manufacturing, Vol. 17, 2001, pp.
355–365.
