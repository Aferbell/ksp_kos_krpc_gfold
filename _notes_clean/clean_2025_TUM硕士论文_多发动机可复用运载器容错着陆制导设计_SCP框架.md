# DLR_TUM_FaultTolerant_Landing_Guidance_MultiEngine_RLV_thesis

- source_file: `DLR_TUM_FaultTolerant_Landing_Guidance_MultiEngine_RLV_thesis.pdf`
- pages: 114
- pdf_title_meta: Master-Thesis
- pdf_author_meta: Giuseppe Soldati (TUM)
- chars_extracted: 173869

## Detected headings (heuristic)

- Giuseppe Soldati
- Master Thesis
- Design of Fault-Tolerant Landing Guidance for Multi-Engine
- Reusable Launchers
- Statutory Declaration
- Giuseppe Soldati
- Signature
- Acknowledgements
- Abstract
- Contents
- Contents
- List of Figures
- List of Tables
- List of Symbols
- Introduction
- Thesis Overview
- Methodology
- Mathematical Preliminaries
- Convex Optimization
- Second Order Cone Programming
- Optimal Control
- Modeling
- Aerodynamic Characteristics
- Chair of Space Mobility and Propulsion
- Technical University Munich
- Contents
- Boundary Conditions
- Initialization
- Scaling
- Nominal Trajectory Solution
- Online Guidance
- Initialization
- Initialization
- Implementation Details
- Setup
- Future Work
- Bibliography
- Chair of Space Mobility and Propulsion
- Technical University Munich
- List of Figures

---

## Full text

The present work was submitted to the Chair of Space Mobility and Propulsion presented by
Giuseppe Soldati
Student ID No.: 03773828
Master Thesis
Design of Fault-Tolerant Landing Guidance for Multi-Engine
Reusable Launchers
Munich, October 6, 2025
Supervising professor: Prof. Dr.-Ing. Chiara Manfletti
Assistant supervisor:
Felix Ebert, M.Sc.
Stefano Far`ı, M.Sc.
Dr.-Ing. Marco Sagliano
1st examiner:
Prof. Dr.-Ing. Chiara Manfletti

Statutory Declaration
I hereby declare that the present work was written independently by me, and that no other aids than those specified were used. The sections of the work that have been taken from other works, either verbatim or in essence, are clearly marked as such in each individual case, with an indication of the source. This declaration also extends to graphics, drawings, map sketches, and pictorial representations contained in the work.
I also agree that my Master’s, Bachelor’s, or term paper may be made accessible by the department to professionally interested persons upon request, including through a library, and that the results contained therein, as well as the developments and programs created during this work, may be used without restriction by the Chair of Space Mobility and Propulsion. (Rights to any programs and inventions that may arise must be clarified in advance.)
Name:
Giuseppe Soldati
Matr. Nr.:
03773828
Place, Date
Signature

Acknowledgements
This remarkable experience is coming to an end, and I want to take a moment to thank the people who made it possible.
My first thoughts go to Stefano and Marco. Thank you, Stefano, for always finding the time to answer my questions and for showing such dedication and hard work. You have been a true example to me. And thank you, Marco, for your guidance and experience, for always finding solutions to any problem, and for fueling my curiosity and drive to learn.
This journey would not have been possible without Felix, who gave me the opportunity to carry out this thesis and was always available when I needed support.
I’m very grateful to everyone in the Navigation and Control department at the Institute of Space Systems.
And in particular, to all the “student room” friends, you made the office feel like home.
On a personal note, I wish to express my gratitude to my family for their unwavering support and to
Giulia, thank you for your constant presence and for being with me every step of this journey.
Finally, I’d like to thank my lifelong friends from Rimini for all the laughs and fun we have shared, and my friends in Munich for making these master’s years truly unforgettable.

Abstract
This work investigates whether, and in what manner, online guidance can provide fault tolerance in the presence of propulsion faults during the landing burn, and to determine the extent to which this contingency guidance remains effective throughout the descent. To this end, the effect of guidance reconfiguration is analyzed in the presence of engine loss, thrust inefficiencies, and thrust vectoring faults.
A return-to-launch-site scenario is considered, restricted to the landing phase, where the mitigation of engine faults is paramount to recover the vehicle. In such cases, a new optimal trajectory must be recomputed, constraining the solution to the new degraded vehicle resources.
To address these challenges, a fault-tolerant landing guidance method for a multi-engine Reusable Launch
Vehicle is developed, formulated through a sequential convex programming framework, where the optimization problem is updated according to the fault information.
Monte Carlo simulations demonstrate that the algorithm is highly effective in generating feasible recovery trajectories under a wide range of fault scenarios. The results show that the proposed method allows successful recovery where nominal guidance would otherwise fail.

Contents
Contents
List of Figures
VII
List of Tables
IX
List of Symbols
XI
1
Introduction
1
1.1
Reusable Launch Vehicles: Historical Context . . . . . . . . . . . . . . . . . . . . . . . . .
1
1.1.1
Fault-Tolerant Guidance: Motivation and Background . . . . . . . . . . . . . . . .
3
1.2
Guidance Algorithms: State of the Art . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
4
1.2.1
Offline Optimization . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
4
1.2.2
Online Optimization . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
4
1.3
Research Questions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
5
1.4
Thesis Overview
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
6
2
Methodology
7
2.1
Methodology for Research Question 1
. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
7
2.2
Methodology for Research Question 2
. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
8
3
Mathematical Preliminaries
9
3.1
Parametric Optimization . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
9
3.1.1
Nonlinear Optimization . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
9
3.1.2
Convex Optimization
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
9
3.1.3
Second Order Cone Programming
. . . . . . . . . . . . . . . . . . . . . . . . . . .
11
3.2
Optimal Control
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
12
3.2.1
Indirect Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
13
3.2.2
Direct Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
13
4
Modeling
19
4.1
Recovery Strategies for Reusable Launch Vehicles . . . . . . . . . . . . . . . . . . . . . . .
19
4.2
Vehicle Configuration & Mission Profile
. . . . . . . . . . . . . . . . . . . . . . . . . . . .
19
4.3
Equations of Motion . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
21
4.3.1
Aerodynamic Characteristics
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
24
4.3.2
Propulsive Characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
26
4.4
Equations of Motion Summary and Assumptions . . . . . . . . . . . . . . . . . . . . . . .
27
Chair of Space Mobility and Propulsion
|
Technical University Munich
V

Contents
5
Nominal Guidance & Control
29
5.1
Nominal Guidance . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
29
5.1.1
State and Control Augmentation . . . . . . . . . . . . . . . . . . . . . . . . . . . .
29
5.1.2
Path Constraints . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
5.1.3
Boundary Conditions
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
5.1.4
Objective Function . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
5.1.5
Problem Statement . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
32
5.1.6
Initialization
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
32
5.1.7
Scaling
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
32
5.1.8
Nominal Trajectory Solution
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
33
5.1.9
Solution Verification . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
33
5.2
Nominal Controller . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
37
5.2.1
Linear Analysis of the Closed-Loop System . . . . . . . . . . . . . . . . . . . . . .
39
5.2.2
Nonlinear Analysis . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
40
6
Online Guidance
45
6.1
Sequential Convex Programming: Theoretical Overview . . . . . . . . . . . . . . . . . . .
45
6.2
Algorithm Derivation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
46
6.2.1
Convexification Procedure . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
48
6.2.2
Discretization . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
51
6.2.3
Convergence Check . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
54
6.2.4
Initialization
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
54
6.2.5
Nominal SCP Validation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
56
6.2.6
Feasibility Check . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
60
6.3
Fault-Tolerant Sequential Convex Programming . . . . . . . . . . . . . . . . . . . . . . . .
60
6.3.1
Fault Scenarios . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
62
6.3.2
Final Position Relaxation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
63
6.3.3
Box Constraints & Final Boundary Condition Update . . . . . . . . . . . . . . . .
63
6.3.4
Initialization
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
64
6.3.5
Performance Under Fault Scenarios . . . . . . . . . . . . . . . . . . . . . . . . . . .
64
7
Verification & Validation
73
7.1
Implementation Details
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
73
7.2
Study 1: Single Faults . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
73
7.2.1
Goal . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
73
7.2.2
Setup
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
74
7.3
Study 2: Multiple Faults . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
84
8
Conclusions & Future Directions
87
8.1
Limitations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
88
8.2
Future Work
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
88
Bibliography
91
Chair of Space Mobility and Propulsion
|
Technical University Munich
VI

List of Figures
List of Figures
1.1
Examples of VTVL vehicles. From the left: (a) Xombie demonstrates the G-FOLD algorithm (2013). (b) Blue Origin’s New Shepard first vertical landing (2015). (c) SpaceX’s
Super Heavy booster performs the first successful catch landing with the Mechazilla arms (2024). . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
2
3.1
Second order cone constraint in R3: ∥u∥2 ≤t. . . . . . . . . . . . . . . . . . . . . . . . . .
12
3.2
Multiple shooting defects. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
17
4.1
Recovery strategies for RLVs. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
20
4.2
Vehicle body axes and aerodynamic angles.
. . . . . . . . . . . . . . . . . . . . . . . . . .
21
4.3
Reference frames: inertial UEN frame I centered at the landing pad and body–fixed frame
B.
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
22
4.4
Vertical flight-path and azimuth angles.
. . . . . . . . . . . . . . . . . . . . . . . . . . . .
24
4.5
Cross-section of the engine cluster. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
26
5.1
GPOPS states: position, velocity, and mass. . . . . . . . . . . . . . . . . . . . . . . . . . .
34
5.2
GPOPS controls. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
35
5.3
Optimality check: reconstructed Hamiltonian. . . . . . . . . . . . . . . . . . . . . . . . . .
36
5.4
Validation of GPOPS solutions: discrepancy between Runge-Kutta and GPOPS states. .
37
5.5
Validation of GPOPS solutions: position error over time.
. . . . . . . . . . . . . . . . . .
38
5.6
Block diagram: nominal closed-loop. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
39
5.7
Closed-loop stability analysis: Nichols charts. . . . . . . . . . . . . . . . . . . . . . . . . .
40
5.8
Frequency domain analysis. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
41
5.9
Time domain analysis: step responses for x- and y-position. . . . . . . . . . . . . . . . . .
42
5.10 Closed-loop Monte Carlo campaign: successful landings. . . . . . . . . . . . . . . . . . . .
43
6.1
SCP algorithm schematic. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
46
6.2
Comparison of position, velocity and mass states. . . . . . . . . . . . . . . . . . . . . . . .
57
6.3
Comparison of propulsive quantities: thrust magnitudes, TVC deflections, and rates. . . .
58
6.4
Comparison of aerodynamic quantities: aerodynamic angles and rates. . . . . . . . . . . .
59
6.5
Evolution of the cost function components across SCP iterations. . . . . . . . . . . . . . .
60
6.6
Validation of SCP solution: discrepancy between Runge-Kutta and SCP states. . . . . . .
61
6.7
FT-SCP algorithm: integration of fault information. . . . . . . . . . . . . . . . . . . . . .
65
6.8
FT-SCP solution under F1 (total thrust loss): position, velocity, and mass evolution. . . .
66
6.9
FT-SCP solution under F1 (total thrust loss): controls evolution. . . . . . . . . . . . . . .
67
6.10 FT-SCP solution under F2 (thrust degradation): position, velocity, and mass evolution. .
68
6.11 FT-SCP solution under F2 (thrust degradation): controls evolution.
. . . . . . . . . . . .
69
Chair of Space Mobility and Propulsion
|
Technical University Munich
VII

List of Figures
6.12 FT-SCP solution under F3 (TVC jamming): position, velocity, and mass evolution. . . . .
70
6.13 FT-SCP solution under F3 (TVC jamming): controls evolution. . . . . . . . . . . . . . . .
71
7.1
F1 (total thrust loss): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
75
7.2
F1 (total thrust loss): 3D landing trajectories of all converged cases. . . . . . . . . . . . .
75
7.3
F1 (total thrust loss): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
76
7.4
F1 (total thrust loss): Final mass at landing versus fault time for all converged cases.
. .
77
7.5
F1 (total thrust loss): FT-SCP convergence analysis. . . . . . . . . . . . . . . . . . . . . .
78
7.6
F1 (total thrust loss): State errors at fault time for converged and non-converged cases.
.
78
7.7
F2 (thrust degradation): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
79
7.8
F2 (thrust degradation): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
79
7.9
F3 (TVC jamming): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
80
7.10 F3 (TVC jamming): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
81
7.11 F3 (TVC jamming): FT-SCP convergence analysis. . . . . . . . . . . . . . . . . . . . . . .
82
7.12 F3 (TVC jamming) relaxed solution: position and velocity.
. . . . . . . . . . . . . . . . .
82
7.13 F3 (TVC jamming) relaxed solution: controls. . . . . . . . . . . . . . . . . . . . . . . . . .
83
7.14 F2 (thrust degradation on multiple engines): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases. . . . . . . . . . . . . . . . . . . . . .
84
7.15 F4 (thrust degradation on multiple engines): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases. . . . . . . . . . . . . . . . . . . . . .
85
7.16 F4 (thrust degradation on multiple engines): Critical thrust as a function of fault time.
.
86
Chair of Space Mobility and Propulsion
|
Technical University Munich
VIII

List of Tables
List of Tables
4.1
Rocket parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
20
5.1
NLP integral weights . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
5.2
NLP box constraints and boundary conditions . . . . . . . . . . . . . . . . . . . . . . . . .
31
5.3
Scaling of fundamental and derived quantities . . . . . . . . . . . . . . . . . . . . . . . . .
33
5.4
Initial conditions and parameter uncertainties . . . . . . . . . . . . . . . . . . . . . . . . .
43
5.5
Successful landing requirements . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
43
6.1
SCP Optimization variables . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
54
6.2
SCP parameters
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
56
6.3
Integral values comparison . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
57
6.4
Failure Cases . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
62
6.5
Fault information bus
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
62
7.1
Randomized parameters for single-fault cases F1–F3 . . . . . . . . . . . . . . . . . . . . .
74
7.2
Representative failed case (F3): Initial conditions and fault parameters . . . . . . . . . . .
82
7.3
Randomized parameters for multiple-fault case F4
. . . . . . . . . . . . . . . . . . . . . .
84
Chair of Space Mobility and Propulsion
|
Technical University Munich
IX

List of Symbols
List of Symbols
General Symbols
˙m
[kg/s]
Mass flow rate
˙T
[N/s]
Thrust rate r
[m]
Position vector u
Control vector v
[m/s]
Velocity vector x
State vector
τ
Normalized time
A
[m2]
Area g0
[m/s2]
Standard gravity
Isp
[s]
Specific impulse
J
Objective / cost m
[kg]
Mass
Ma
[-]
Mach number sη
[-]
Trust region slack variable sR
[-]
Integral slack variable sκ
[-]
Virtual controls slack variable spos
[-]
Final position slack variable
Sref
[m2]
Reference aerodynamic surface
T
[N]
Thrust t
[s]
Time
V
[m3]
Volume
Greek Symbols
Chair of Space Mobility and Propulsion
|
Technical University Munich
XI

List of Symbols
α
[◦]
Angle of attack
αeff
[◦]
Effective angle of attack
αtail
[◦]
Tail angle of attack
β
[◦]
Sideslip angle
η
[-]
Trust region vector
κ
[-]
Virtual controls slack vector
λ
[-]
Costate vector
ν
[-]
Virtual controls vector
∆τ
[-]
Normalized step size
γv
[◦]
Vertical flight-path angle
µ
[m3/s2]
Earth’s gravitational parameter
ψv
[◦]
Vertical azimuth angle
ρ
[kg/m3]
Atmospheric density
Subscripts
B
Body frame
CoM
Center of mass
CoP
Center of pressure
I
Inertial frame k
Quantity at time step k
NED
North–East–Down frame ref
Reference / nominal
UEN
Up–East–North frame
W
Wind frame
Superscripts n
Quantity at iteration n
T
Transpose
Coefficients
CA
[-]
Axial aerodynamic coefficient
Chair of Space Mobility and Propulsion
|
Technical University Munich
XII

List of Symbols
CN
[-]
Normal aerodynamic coefficient
Cx
[-]
Body-axis aerodynamic coefficient (x)
Cy
[-]
Body-axis aerodynamic coefficient (y)
Cz
[-]
Body-axis aerodynamic coefficient (z)
Special Symbols
∥·∥2
Euclidean norm
Abbrevations
CoM
Center of Mass
CoP
Center of Pressure
DCM
Direction Cosine Matrix
DRL
Downrange Landing
FDD
Fault Detection and Diagnosis
FTC
Fault-Tolerant Control
FTG
Fault-Tolerant Guidance
LQR
Linear Quadratic Regulator
LTI
Linear Time Invariant
NED
North-East-Down
NLP
Nonlinear Program
OCP
Optimal Control Problem
PDL
Powered Descend and Landing
RLV
Reusable Launch Vehicle
RTLS
Return To Launch Site
SCP
Sequential Convex Programming
SCvx
Successive Convexification
SOC
Second Order Cone
SOCP
Second Order Cone Programming
TVC
Thrust Vector Control
UEN
Up–East–North
VTVL
Vertical Takeoff Vertical Landing
Chair of Space Mobility and Propulsion
|
Technical University Munich
XIII

List of Symbols
Chair of Space Mobility and Propulsion
|
Technical University Munich
XIV

1. Introduction
1 Introduction
1.1 Reusable Launch Vehicles: Historical Context
Space transportation is experiencing a significant change, as substantial global efforts are directed toward the development of Vertical Takeoff Vertical Landing (VTVL) Reusable Launch Vehicles (RLVs).
Reusable rockets are highly valuable assets for the organizations that have the expertise to operate them [9].
The idea of vertical landing dates back to the very beginning of spaceflight. The first spacecraft to achieve a survivable landing on a celestial body was Luna 9, which successfully touched down on the Moon on
February 3, 1966 [79], after multiple previous Soviet attempts. This marked the first human-made object to achieve a soft landing on another celestial body. The Soviets later expanded this success with Venera
7, performing the first soft landing on Venus on December 15, 1970 [76].
On the U.S. side, NASA’s Surveyor I mission achieved the first American soft landing on the Moon on June 2, 1966 [31], employing a three-legged landing structure and retro-propulsion, unlike the Soviet
Luna landers. Later, Viking I became the first spacecraft to successfully soft-land on Mars on July 20,
1976 [54].
On Earth, interest in reusable rockets was already explored in the Apollo period and resurfaced with the
Space Shuttle program [9]. Recovery concepts can be broadly grouped into two categories: those aiming for a vertical landing of the reusable stage, and those pursuing horizontal landing [26]. The latter include the U.S. Shuttle itself or Europe’s Hermes spaceplane, although the project was canceled before it ever flew [30]. More recently, uncrewed winged demonstrators such as the Deutsche Zentrum f¨ur Luft- und
Raumfahrt (DLR) Reusability Flight Experiment (ReFEx) [10] continue this line of development.
One of the first demonstrations of vertical landing on Earth was achieved in 1993 by the McDonnell
Douglas Delta Clipper Experimental (DC-X ) vehicle [34]. A key milestone followed with the G-FOLD
(Guidance for Fuel-Optimal Large Diverts) algorithm [5], first demonstrated in 2013 on the Masten Space
Systems Xombie VTVL suborbital rocket [74] (see Figure 1.1a), showing the feasibility of onboard realtime computation of large divert maneuvers. Shortly thereafter, private companies began demonstrating
VTVL at scale. Blue Origin successfully recovered its New Shepard suborbital booster in November 2015
[17] (Figure 1.1b), later reusing it, while SpaceX recovered its first Falcon 9 stage in December 2015 [91]
and reflown it in 2017.
Both companies are now advancing towards heavy-lift reusable launch systems. Blue Origin is developing
New Glenn, a medium-to-heavy lift rocket with a reusable first stage whose maiden flight occurred on
January 16, 2025 [18], reaching orbit though the booster landing attempt failed. SpaceX is developing
Starship and its Super Heavy booster, both fully reusable and capable of autonomous vertical landing.
Chair of Space Mobility and Propulsion
|
Technical University Munich
1

1. Introduction
On October 13, 2024, the Super Heavy booster demonstrated the first successful “catch” landing using mechanical arms [84] (Figure 1.1c).
Figure 1.1: Examples of VTVL vehicles. From the left: (a) Xombie demonstrates the G-FOLD algorithm (2013). (b) Blue Origin’s New Shepard first vertical landing (2015). (c) SpaceX’s Super Heavy booster performs the first successful catch landing with the Mechazilla arms (2024).
Globally, multiple space agencies are pursuing similar technologies. China is developing a reusable variant of its Long March 8 [42], while India is progressing with the Reusable Launch Vehicle–Technology
Demonstrator (RLV-TD) [2], aimed at hypersonic flight and autonomous landing validation. In Europe, the European Space Agency (ESA) and ArianeGroup are developing Themis, a methane-powered VTVL
demonstrator [1], while DLR, the Centre National d’´Etudes Spatiales (CNES), and the Japan Aerospace
Exploration Agency (JAXA) are collaborating on CALLISTO (Cooperative Action Leading to Launcher
Innovation in Stage Toss-back Operations) [29], a small-scale VTVL platform that serves to demonstrate the manoeuvres and operations required for a reusable first stage, together with validating the potential economic benefits such a system could bring to the European space sector.
We can see that the renewed interest in rocket reusability in recent years is translating into numerous products and prototypes that are demonstrating reliable performance. Yet, these efforts may not be directed toward developing methods for fault compensation [32]. An industry-wide study of international launches examined 118 failures from 2000 to mid-2024 and analyzed their statistical distribution across different categories, finding that propulsion issues account for the largest share of mission losses (49.6%)
[38]. Consistent with this trend, another analysis reports that propulsion failures represent 54% of cases over the last 15 years [33].
Examples of failed landing attempts due to propulsion system failures include: (i) the Falcon 9 landing attempt on June 15, 2016, where one of the three Merlin engines produced less thrust than expected during the terminal burn, resulting in a crash on the droneship with a consequent loss of the vehicle [22]; and (ii) the Falcon Heavy test flight on February 6, 2018, in which the center core aimed for a droneship return but failed to ignite two of its three engines for the landing burn [23].
Chair of Space Mobility and Propulsion
|
Technical University Munich
2

1. Introduction
1.1.1 Fault-Tolerant Guidance: Motivation and Background
A fault is defined as any unpermitted deviation of at least one characteristic property or parameter of a system from its standard condition [40]. Fault Detection and Isolation (FDI) consists of determining whether a fault is present and, if so, locating and characterizing it. It generally relies on redundancy, which can be provided either by hardware or by analytical methods. Analytical redundancy employs a mathematical model of the system together with estimation techniques, and can be classified into quantitative model-based approaches, which use explicit mathematical models to generate residuals, or qualitative model-based methods, which use artificial intelligence (AI)[39].
On the other hand, hardware redundancy achieves fault detection mainly through cross checks, consistency checks or voting mechanisms [93], for example by comparing measurements of the same signal coming from multiple sensors.
Building upon FDI, Fault Detection, Isolation, and Reconfiguration (FDIR) is a control strategy that maintains safe or acceptable system operation in the presence of faults [39]. Assuming a correct and timely FDI, fault-tolerant control (FTC) enables the system to maintain stability and a limited level of performance despite the reduced resources caused by the fault. This is achieved by using the remaining control authority, allowing recovery from adverse flight conditions caused by faults [80]. For instance, an adaptive controller which generates new control parameters online after a fault occurs is discussed in
[56], while [59] presents several strategies for handling thrust and TVC failures depending on the severity of the fault. These include switching to a different control allocation scheme, selecting a precomputed control law designed to be robust against failures, and reconfiguring the TVC inner-loop controller.
When flight conditions are too degraded for FTC to be effective, fault-tolerant guidance (FTG) becomes necessary. FTG refers to a redefinition of the onboard mission objectives when failures render the original plan infeasible. For example, recovery strategies for reentry missions are examined in [52]. While
[57] describes online reconfiguration in the presence of a mid-flight change in the performance of the propulsion system of a Vertical Take Off and Landing (VTOL) aircraft. The work in [82] motivates FTG
as a recovery strategy for failures that conventional techniques cannot handle, this can be achieved by changing mission objectives and recomputing a feasible trajectory accounting for the reduced resources, reshaping the mission autonomously to avoid the total loss of a launcher during ascent. Along these lines, real-time trajectory replanning for multistage launch vehicles under faults such as thrust drop or mass flow anomalies is studied in [51, 83]. Here, online optimal control is applied to generate a feasible trajectory after a fault occurs. Trajectory regeneration is carried out online, with the strategy determined by the severity of the fault: for less severe cases, the mission goal is preserved but the trajectory is adapted to remain feasible given the degraded resources, while for more severe cases the mission target itself is modified, switching from the original target orbit to a safe rescue orbit. Finally, when FTC is insufficient to address severe faults, FTG methods such as those described in [59] allow the continuation of a mission by regenerating an updated reference trajectory that explicitly accounts for the fault, by computing a feasible and locally optimal solution around the original nominal trajectory but consistent with the new faulty dynamics.
Chair of Space Mobility and Propulsion
|
Technical University Munich
3

1. Introduction
1.2 Guidance Algorithms: State of the Art
Computing guidance means generating a dynamically feasible state and control trajectory that satisfies a set of constraints while optimizing a mission objective, and such a problem is naturally formulated as an optimal control problem (OCP). All terms introduced here are meant in a broad sense, and their precise meaning will be clarified in the remainder of this work.
Trajectory optimization problems can be distinguished between offline and online. Offline formulations typically rely on indirect optimal control approaches or on direct transcription methods that result in nonlinear programming (NLP) problems. These approaches are categorized as offline because general
NLP problems provide no guarantees of finding a feasible solution, may converge only to local minima, and are often sensitive to initial guesses [55]. As a result, they cannot reliably ensure safe guidance in real time. Furthermore, unpredictable computational time and the absence of assured algorithm convergence exclude the use of such methods for real-time applications, which demand both reliability and fast onboard solutions [45]. In contrast, trajectory optimization methods are classified as online-capable if they can provide onboard guidance commands in real time that satisfy the problem constraints and terminal conditions, independent of precomputed reference trajectories [81].
These methods generally rely on direct formulations that exploit convexity to achieve predictable runtimes and guaranteed convergence properties.
1.2.1 Offline Optimization
A number of software packages have been developed to address offline optimal control problems (OCPs).
FALCON.m [61] provides an object-oriented framework within MATLAB [89] for modeling and solving general OCPs. Another widely used tool is GPOPS-II [58], which implements variable-order Gaussian quadrature collocation methods to transcribe the continuous-time problem into a sparse nonlinear program.
DLR has developed SPARTAN [71], a tool based on pseudospectral transcription capable of handling complex multi-phase problems. Other well-established packages include DIDO [62], which enabled the International Space Station (ISS) to execute an optimal maneuver that saved one million dollars and marked the first in-flight use of pseudospectral methods [12], and CASADI [7], a flexible framework for algorithmic differentiation and numerical optimization that has been increasingly adopted for optimal control applications.
1.2.2 Online Optimization
Unlike nonconvex approaches, convex problems can be solved to global optimality under mild assumptions, with guaranteed convergence in polynomial time [20, 41]. This property makes convex optimization especially suitable for real-time applications where reliability and predictability of solve times are critical. Unfortunately, many practical trajectory generation problems, such as powered descend and landing (PDL), are inherently nonconvex and thus not directly compatible with convex optimization methods.
Since trajectory generation problems are almost always nonconvex, online guidance methods typically rely on some form of convexification[49].
State of the art algorithms address this challenge by reformulating the original nonconvex guidance problem into one that can be solved using a convex optimizer. Two main strategies exist:
Chair of Space Mobility and Propulsion
|
Technical University Munich
4

1. Introduction
1. Lossless Convexification (LCvx).
LCvx reformulates certain classes of nonconvex (OCPs)
into higher-dimensional convex problems.
Crucially, it can be shown that the solution of the convex problem is also an optimal solution to the original nonconvex problem, without excluding any feasible solutions [4]. This “lossless” property ensures that if a feasible solution exists, the convexified problem will find it [3]. However, LCvx formulations for more realistic lander models, such as full six-degree-of-freedom (6-DoF) dynamics or cases including complex aerodynamic forces, remain underdeveloped, with only simplified examples studied to date.
2. Sequential Convex Programming (SCP). When LCvx cannot be applied, convex optimization can still be used through SCP methods. These approaches iteratively solve a sequence of convex subproblems obtained by linearizing the nonconvexities around the solution at the previous iteration.
Well-known algorithms include Successive Convexification (SCvx) [50] and GuSTO [19], which represent two variants of this general methodology.
Further research has explored hybrid approaches combining pseudospectral optimal control methods with convex optimization techniques.
For example, a framework integrating pseudospectral methods with convex formulations for powered descent guidance was introduced in [65].
Several of these algorithms have already been demonstrated on real missions. LCvx was deployed on the
Masten Space Systems Xombie rocket via the G-FOLD algorithm [5], showing that convex optimization could solve precision landing problems in real time. NASA has also tested SCvx onboard Blue Origin’s
New Shepard as part of the SPLICE (Safe and Precise Landing Integrated Capabilities Evolution) program
[53].
Lastly, the European CALLISTO demonstrator is set to carry convex optimization algorithms onboard for guidance and control [69].
1.3 Research Questions
The objective of this thesis is to propose and analyze the effect of guidance reconfiguration in the presence of engine loss, thrust inefficiencies, or thrust vectoring faults. The study focuses on a return to launch site scenario, limited to the landing phase, where the mitigation of engine faults is paramount to recover the vehicle. To this purpose, a new optimal trajectory must be recomputed while constraining the solution to the degraded vehicle resources, which makes the formulation of the underlying optimization problem critical. The thesis builds upon an RLV mission benchmark based on [77], modified to include a cluster of rocket engines rather than a single-engine configuration. This not only provides additional redundancy in thrust capability but also better aligns to the future needs of commercial vehicles. For these reasons, the first research question is:
Research Question 1: How can an online guidance provide fault tolerance in presence of faults affecting the engines in reusable launch vehicles during the landing burn?
Once a functioning algorithm has been established, the next step is to examine both its robustness and its limits. This means subjecting it to a wide range of fault scenarios and operating conditions in order to identify the envelope within which a safe landing can still be guaranteed.
To this end, extensive
Verification and Validation is carried out, which motivates the second research question.
Research Question 2: In which measure (namely, until when across the descent) is the contingency guidance effective?
Chair of Space Mobility and Propulsion
|
Technical University Munich
5

1. Introduction
1.4 Thesis Overview
Chapter 2 outlines the overall research methodology. Chapter 3 introduces the mathematical preliminaries, beginning with a general classification of optimization problems and concluding with an overview of methods for solving (OCPs). Chapter 4 presents the mission profile and the VTVL model, including the derivation of the equations of motion. Building on this foundation, Chapter 5 defines the reference trajectory, which serves as the basis for the rest of the thesis, and describes the controller used to track it.
The core contribution of the work begins in Chapter 6, where the convex formulation of the guidance algorithm is derived in detail. This formulation is subsequently extended to account for engine and thrust vectoring faults, directly addressing the first research question. Chapter 7 answers the second research question by testing the algorithm through extensive Monte Carlo simulations, leading to conclusions on the impact of faults on mission success and the validation of the algorithm’s robustness.
Finally, Chapter 8 states the conclusions of the thesis and points out directions for future work.
Chair of Space Mobility and Propulsion
|
Technical University Munich
6

2. Methodology
2 Methodology
The objective of this chapter is to give an overview on the decisions that have led to the choices made throughout the thesis, how the models, algorithms, and simulation environment are developed in order to addresses the research questions posed in Chapter 1.
Multiple surveys have shown that propulsion-related anomalies represent one of the most frequent causes of launch vehicle failure. For this reason, this work focused on propulsion and TVC related failures.
2.1 Methodology for Research Question 1
Research Question 1 (RQ1): How can an online guidance provide fault tolerance in presence of faults affecting the engines in reusable launch vehicles during the landing burn?
The formulation of RQ1, which explicitly requires an online solution, motivated the investigation of guidance algorithms suitable for real-time implementation, leading to the choice of a convex method. As with all OCPs, the process began with the definition of an appropriate flight dynamics model.
A 3-DoF translational model with a cluster of engines was selected. The multi-engine setup is essential to allow for fault-tolerance, as one engine would not be enough, while the 3-DoF formulation was preferred over a full 6-DoF model for two reasons: (i) real-time applicability requires reduced computational burden, and (ii) the simplified translational model coupled with the engine cluster had not yet been explored in the literature.
Once the nonlinear dynamics had been defined, a convexification strategy was required.
Among the available methods, LCvx could not be applied, as existing formulations do not cover the selected flight dynamics model. Therefore, SCP was adopted, since it is a more general method able to tackle the nonconvexities of the problem while still offering opportunities for further development beyond the scope of this thesis.
The guidance problem was thus formulated through SCP, where nonlinearities are linearized at each iteration until convergence is achieved. Details of the algorithm are provided in Chapter 6. First, a standard
SCP algorithm was developed, and its correctness was verified by comparing its solutions with those obtained from the well-known OCP solver GPOPS-II [58]. The convex optimization problems that are solved in the SCP framework in each iteration were written using CVX, a MATLAB library for specifying and solving convex programs [24, 35] and solved using the convex optimization solver MOSEK [8]. Once the baseline SCP algorithm was validated, it was then augmented to provide guidance reconfiguration in the presence of faults. This was achieved by appropriately updating the OCP components based on the fault acting on the system. The resulting algorithm, named Fault-Tolerant Sequential Convex Programming (FT-SCP), was then tested on a range of representative cases for all the considered faults, thus addressing RQ1.
Chair of Space Mobility and Propulsion
|
Technical University Munich
7

2. Methodology
2.2 Methodology for Research Question 2
Research Question 2 (RQ2): To what extent, i.e., until when across the descent, is the contingency guidance effective?
Answering RQ2 served multiple purposes. First, it enabled a robustness assessment of the algorithm developed to address RQ1 with respect to dispersions in initial conditions. Second, it allowed an investigation of how the occurrence of different propulsion faults at different times during the descent affects the landing success. RQ2 was addressed in Chapter 7 through a series of Monte Carlo campaigns that combined the FT-SCP algorithm with a feedback controller designed to track the computed trajectories.
For this purpose, a simple Linear Quadratic Regulator (LQR) was employed, since the focus of the thesis lies on the guidance aspect rather than on controller design.
In conclusion, this chapter outlined the motivation behind the choices made to address the research questions, which will be explained in greater detail in the following chapters.
Chair of Space Mobility and Propulsion
|
Technical University Munich
8

3. Mathematical Preliminaries
3 Mathematical Preliminaries
This chapter introduces the mathematical background required throughout the thesis. It starts with basic notions from optimization theory and then moves to the formulation of OCPs and their numerical solution methods.
3.1 Parametric Optimization
3.1.1 Nonlinear Optimization
Nonlinear optimization, or nonlinear programming (NLP) problems take the general mathematical form [11].
Let z = (z1, . . . , zn) ∈Rn denote the vector of decision variables. The problem can be written as minimize f(z)
(objective function)
subject to gi(z) ≤0, i = 1, . . . , m (inequality constraints)
hj(z) = 0, j = 1, . . . , p (equality constraints)
(3.1)
where g(z) =


g1(z)
...
gm(z)

∈Rm, h(z) =


h1(z)
...
hp(z)

∈Rp.
(3.2)
and f : Rn →R, g : Rn →Rm, h : Rn →Rp.
(3.3)
The objective of problem (3.1) is to determine the variables z1, . . . , zn that satisfy the equality and inequality constraints defined by h and g, while minimizing the objective function f.
3.1.2 Convex Optimization
Before discussing what makes an optimization problem convex, it is useful to introduce the basic components of such problems. We begin with the definition of convex sets, followed by the notion of convex functions, and finally arrive at the structure of a convex optimization problem.
Let z1, z2 ∈Rn. Any combination of these points of the form z = λ1z1 + λ2z2
(3.4)
is also an element of Rn. This concept extends naturally to any number of vectors i = 1, . . . , K.
Chair of Space Mobility and Propulsion
|
Technical University Munich
9

3. Mathematical Preliminaries
Depending on what restrictions we place on the coefficients λi, we obtain different types of combinations
[20]:
1. Linear combination: coefficients are arbitrary real numbers z = λ1z1 + λ2z2,
λi ∈R
(3.5)
2. Affine combination: coefficients sum to one z = λ1z1 + λ2z2,
λi ∈R,
λ1 + λ2 = 1
(3.6)
3. Convex combination: coefficients are nonnegative and sum to one z = λ1z1 + λ2z2,
λ1 + λ2 = 1,
λi ≥0
(3.7)
4. Conic combination: coefficients are nonnegative z = λ1z1 + λ2z2,
λi ≥0
(3.8)
A set C ⊂Rn is affine if, for any z1, z2 ∈C and λ ∈R,
λz1 + (1 −λ)z2 ∈C
(3.9)
That is, C contains all affine combinations of its points [20].
A set C ⊂Rn is convex if, for any z1, z2 ∈C and λ1, λ2 ≥0 with λ1 + λ2 = 1,
λ1z1 + λ2z2 ∈C
(3.10)
Equivalently, C includes the line segment connecting any two of its points [20].
Convexity is preserved under several operations [37]. Let C1, C2 ⊂Rn be convex sets, then:
• Intersection:
C1 ∩C2 is convex (3.11)
• Algebraic sum:
C1 + C2 = {z ∈Rn : z = z1 + z2, z1 ∈C1, z2 ∈C2}
is convex (3.12)
• Affine transformation: for f(z) = Az + b, with A ∈Rm×n and b ∈Rm, f(C1) = {y ∈Rm : y = Az1 + b, z1 ∈C1}
is convex (3.13)
Having discussed convex sets and operations that preserve convexity, it is natural to extend the concept
Chair of Space Mobility and Propulsion
|
Technical University Munich
10

3. Mathematical Preliminaries to functions defined on these sets. A function f : C →R, defined on a convex set C ⊂Rn, is said to be convex [20] if, for all z1, z2 ∈C and λ ∈[0, 1], f(λz1 + (1 −λ)z2) ≤λf(z1) + (1 −λ)f(z2).
(3.14)
In the context of the general nonlinear programming problem introduced in (3.1), the optimization problem is classified as convex when its components satisfy the following conditions [20]:
• The objective function f(z) is convex.
• Each inequality constraint function gi(z) is convex.
• Every equality constraint is affine hj(z) = aT
j z −bj, j = 1, . . . , p, (3.15)
where aj ∈Rn and bj ∈R.
3.1.3 Second Order Cone Programming
Second-order cone programming (SOCP) is a subclass of convex optimization in which the objective function is linear, subject to a combination of linear equality constraints, linear inequality constraints and second-order cone (SOC) constraints. A generic SOCP can be written as [20]
min z∈Rn f ⊤z s.t.
∥Aiz + bi∥2 ≤c⊤
i z + di, i = 1, . . . , m,
Fz = g
Gz ≤h (3.16)
Ai ∈Rℓ×n, bi ∈Rℓ, ci ∈Rn, di ∈R,
F ∈Rp×n, g ∈Rp,
G ∈Rq×n, h ∈Rq.
(3.17)
In problem (3.16), the constraints consist of SOC constraints of the form ∥Aiz+bi∥2 ≤c⊤
i z+di, together with linear equality constraints Fz = g and linear inequality constraints Gz ≤h. Each SOC constraint is of dimension ℓ+ 1, where the standard second-order cone is defined as
Kℓ+1 =
n (u, t) ∈Rℓ, t ∈R
 ∥u∥2 ≤t o
.
(3.18)
Thus, each SOC constraint can equivalently be written as [47]

Ai c⊤
i

z +

bi di

∈Kℓ+1, i = 1, . . . , m.
(3.19)
This condition defines a second-order cone in Rℓ+1 for the variables (u, t) obtained through the affine transformation u = Aiz + bi, t = c⊤
i z + di.
(3.20)
Chair of Space Mobility and Propulsion
|
Technical University Munich
11

3. Mathematical Preliminaries
A graphical interpretation is given in Figure 3.1: in the case ℓ= 2, the admissible pairs (u, t), with u ∈R2, occupy the region lying above the boundary of a cone in R3.
Figure 3.1: Second order cone constraint in R3: ∥u∥2 ≤t.
3.2 Optimal Control
The trajectory design goal is to determine a sequence of control inputs such that a given performance index is minimized, while ensuring that the resulting state trajectory evolves according to the differential equation governing the system dynamics. Both the state and control trajectories must satisfy boundary conditions and path constraints.
The problem can be formulated as an OCP, whose fundamental building blocks are:
• The state vector x(t), which defines the state of the system at time t.
• The control vector u(t), which allows to influence the dynamic behavior of the system at time t.
• The system dynamics defined by the differential equation
˙x(t) = f(t, x(t), u(t)), which governs the evolution of the state.
• The objective function to be minimized.
• State and control constraints that must be satisfied along the trajectory.
• The boundary conditions.
Chair of Space Mobility and Propulsion
|
Technical University Munich
12

3. Mathematical Preliminaries
The optimal control problem is formally stated as [60]:
Minimize
ϕ
x(t0), x(tf), t0, tf

+
Z tf t0
f0
t, x(t), u(t)

dt subject to
˙x(t) = f
t, x(t), u(t)

c
t, x(t), u(t)

≤0
ψ
x(t0), x(tf), t0, tf

= 0
(3.21)
OCPs must be solved numerically, and the available numerical methods are commonly divided into two groups: indirect and direct methods, which are described in the following sections.
3.2.1 Indirect Methods
Indirect methods follow a ”first optimize, then discretize” approach, they solve optimal control problems by first applying the necessary conditions for optimality from Pontryagin’s Minimum Principle [16]. This leads to a boundary value problem (BVP) involving both the state and adjoint differential equations.
The main idea is to construct the Hamiltonian, apply the necessary optimality conditions, and then solve the resulting BVP numerically, typically using single or multiple shooting methods [14].
While indirect methods can produce highly accurate solutions, they are often very sensitive to initial guesses and can struggle with path inequalities [15]. In practice, it can be useful to combine direct and indirect methods, for example, using a direct method first to generate a good initial guess for the indirect method.
Indirect methods are outside the scope of this thesis. This section serves only as a brief introduction, and they will not be discussed further.
3.2.2 Direct Methods
Direct methods approximate the state and/or controls of the OCP in a suitable way.
A method is referred to as control parametrization method when only the control is approximated, while it is denoted as state and control parametrization method when both state and control are approximated. In both cases the OCP is discretized and transcribed into a finite-dimensional NLP. A second distinction arises from the manner in which the dynamics are enforced: shooting methods advance the dynamics by numerical simulation of the initial value problem (IVP) on some interval, while collocation methods impose the dynamics as constraints at a set of collocation points [15, 60].
To introduce these methods, we first recall some fundamental notions from the numerical solution of differential equations.
We consider IVPs of the form
˙x(t) = f(x(t), t), x(t0) = x0.
(3.22)
and introduce a time grid t0 < t1 < · · · < tN = tf, ti ∈{t0, . . . , tN}.
(3.23)
Chair of Space Mobility and Propulsion
|
Technical University Munich
13

3. Mathematical Preliminaries
Numerical methods for solving such problems can be mainly divided into one-step methods (e.g., Runge–
Kutta methods [63]) and multi-step methods.
Their distinction lies in what the method exploits to calculate the value of the state at the next time step. For one-step, the value xi+1 depends only on the value xi at the previous grid point. On the other hand, multi-step methods also use previous values xi−1, xi−2, . . . [14].
A common family of one-step methods are the Runge–Kutta methods, which can be defined as [14, 63]:
xi+1 = xi + hi
K
X
j=1
βjfij, (3.24)
where fij = f

xi + hi
K
X
ℓ=1
αjℓfiℓ, ti + hiρj

.
(3.25)
The coefficients ρj, βj, αjℓare known constants, typically organized in the Butcher diagram
ρ1
α11
α12
· · ·
α1k
ρ2
α21
α22
· · ·
α2k
...
...
...
...
...
ρk
αk1
αk2
· · ·
αkk
β1
β2
· · ·
βk
For the trapezoidal method with k = 2, the Butcher tableau is
0
0
0
1
1
2
1
2
1
2
1
2
which leads to xi+1 = xi + hi
2
f(xi, ti) + f(xi+1, ti+1)

.
(3.26)
For simplicity, the following discussion will be restricted to one-step methods, although analogous considerations apply to multi-step methods as well.
The control u must also be discretized on the grid. Common parametrizations include piecewise-constant or piecewise-linear functions, as well as higher-order representations such as B-splines.
For example, a piecewise-constant parametrization results in ui ≈u(ti), ti ∈{t0, . . . , tN}
⇒
uh = (u0, u1, . . . , uN)⊤∈R(N+1)nu.
(3.27)
Given a choice of control parametrization, the state samples xi ≈x(ti) are computed using the one-step method, which produces the state approximation xh = (x0, x1, . . . , xN)⊤∈R(N+1)nx.
(3.28)
Chair of Space Mobility and Propulsion
|
Technical University Munich
14

3. Mathematical Preliminaries
Direct Collocation
Given the discretization of state and control, direct collocation treats both {xi}N
i=0 and {ui}N
i=0 as decision variables, and enforces the dynamics through a set of defect constraints ξi that are imposed on each discretization interval [15].
For example, under trapezoidal discretization, the defect at time step i is given by
ξi = xi+1 −xi −h
2
f(ti, xi, ui) + f(ti+1, xi+1, ui+1)

(3.29)
leading to the following NLP:
min z
J(z)
s.t.
H(z) = 0,
G(z) ≤0.
(3.30)
The equality constraints can be written as
H(z) :=














ξ0 = x1 −x0 −h
2

f(t0, x0, u0) + f(t1, x1, u1)

...
ξN−1 = xN −xN−1 −h
2

f(tN−1, xN−1, uN−1) + f(tN, xN, uN)

Cequality(z)
Ψ(x0, xN)














= 0.
(3.31)
while the inequality constraints are
G(z) :=





c(t0, x0, u0)
...
c(tN, xN, uN)




≤0.
(3.32)
The resulting NLP is large-scale but sparse, a property that many NLP solvers are able to exploit for efficiency [60].
Direct Single Shooting
The fully discretized OCP that results from direct collocation can be reduced in size by integrating the differential equations using any numerical method for IVPs. In fact, by solving the IVP, the state trajectory is fully characterized by the initial state x0 and the control parametrization, leading to the following vector of optimization variables:
z = (x0, u0, . . . , uN)⊤, ui ∈Rnu (3.33)
The direct single shooting method proceeds iteratively.
Starting from an initial guess for the initial state and controls along the trajectory, the system dynamics are integrated from t0 to tf to obtain a candidate trajectory. The resulting state at the final time is then compared with the imposed terminal
Chair of Space Mobility and Propulsion
|
Technical University Munich
15

3. Mathematical Preliminaries state constraint, and the unknown initial state and controls are updated by driving the cost to a lower value. This procedure is repeated until convergence, that is, until the cost is minimized and all constraints are satisfied [60].
Direct Multiple Shooting
The direct single shooting method suffers from convergence problems due to the sensitivity of the states with respect to initial states and preceding control variables. In fact, any slight alteration to these can lead to large, non-linear variations in the subsequent states [14]. To tackle this, in direct multiple shooting the time interval is divided into m + 1 subintervals and the previously mentioned direct single shooting method is used over each subinterval, with the values of the state at the beginning of each shooting segment and the coefficients in the control parametrization being optimization variables [60].
z = (x0, xMS,1, ..., xMS,m, u0, ..., uN)⊤
(3.34)
Furthermore, in order to enforce continuity, the shooting defects are imposed to be zero at each shooting node (see Figure 3.2), such that the terminal state of a segment coincides with the initial state at the segment that follows:
x(t−
j ) = xMS,j−1 +
Z tMS,j tMS,j−1
f(x(t), u(t)) dτ
(3.35)
x(t−
j ) −xMS,j = 0, j = 1, . . . , m (3.36)
where xMS,j denotes the optimization variable for the initial state at shooting node j, and x(t−
j ) is the solution of the IVP obtained by integrating the dynamics over the interval [tMS,j−1, tMS,j] starting from xMS,j−1. The direct multiple shooting method increases the size of the optimization problem, as the state values at the shooting nodes are introduced as optimization variables. However, this larger problem is acceptable because integrating over shorter subintervals decreases sensitivity to errors in the unknown initial conditions. As a result, the states become sensitive only to the preceding shooting nodes and the control variables within each subinterval [60].
Pseudospectral Methods
Pseudospectral (PS) methods are a global form of orthogonal collocation. In this approach, the state is approximated by a global polynomial, and collocation conditions are enforced at specific points, chosen as the roots of orthogonal polynomials [60]. In PS methods, the number of mesh intervals is fixed, while the degree of the polynomial is varied. Increasing the number of collocation nodes p raises the degree of the polynomial approximation, and the solution accuracy improves accordingly. The main benefit of PS
methods is that convergence is spectral (i.e., quasi exponential) with the number of nodes. An additional property of the Gauss pseudospectral method is that the Karush–Kuhn–Tucker (KKT) conditions of the
NLP can be mapped to the costates of the continuous-time OCP [13], a property that will be exploited in Chapter 5.
hp pseudospectral methods extend this idea by dividing the time domain into multiple subdomains, over which the equations of motion are collocated. Two parameters are then defined: h, the number of segments, and p, the number of nodes per segment. These methods were first introduced in optimal
Chair of Space Mobility and Propulsion
|
Technical University Munich
16

3. Mathematical Preliminaries x(t)
x0
t t0
tf
} Defect
XMS,1
XMS,2
XMS,m t1
t2
t3
tm tm-1
XMS,3
XMS,m-1
x(tm
-) = x(tm
+)
Figure 3.2: Multiple shooting defects.
control by [25], who proposed an adaptive version where the number of segments, segment widths, and polynomial degrees are varied. Such a scheme is implemented, for example, in the optimal control software
GPOPS-II [58]. More recently, [64] proposed a hybrid framework combining hp pseudospectral methods with convex optimization, applied to the powered descent and landing (PDL) problem.
Chair of Space Mobility and Propulsion
|
Technical University Munich
17

4. Modeling
4 Modeling
4.1 Recovery Strategies for Reusable Launch Vehicles
Two primary recovery strategies are commonly considered for the first stage of reusable launch vehicles (RLVs): Return to Launch Site (RTLS) and Downrange Landing (DRL) [26] (Figure 4.1).
The DRL scenario is the simplest to execute and the most fuel-efficient, as the reusable stage follows its ballistic or initial trajectory and lands at a designated point, typically an offshore landing platform. The recovery sequence after Main Engine Cut-Off (MECO) can be summarized as follows:
• Tilt-over maneuver: The rocket is reoriented in the direction of flight using gas thrusters.
• Ballistic flight: The rocket follows its ballistic trajectory.
• Entry burn: The engine is reignited to adjust the ballistic trajectory such that the impact point aligns with the landing pad. The engine is then shut down again.
• Aerodynamic phase: The vehicle uses aerodynamic control surfaces to steer and reduce dispersions.
• Powered descent and landing: At a low altitude, the engines are reignited to perform a precise, soft landing.
The RTLS scenario involves a more complex sequence of maneuvers but offers the advantage of eliminating the need for an offshore recovery platform. The primary benefit is that the reusable stage can return and land near the launch site. The recovery sequence following MECO is as follows:
• Tilt-over maneuver: The rocket is reoriented in the direction of flight using gas thrusters.
• Boostback burn: The engine is reignited to reverse the horizontal velocity, redirecting the rocket back toward the launch/landing site.
This is followed by a coast phase, re-entry, and a powered descent and landing phase, which are similar to those in the DRL profile.
4.2 Vehicle Configuration & Mission Profile
The focus of this thesis is on the PDL phase, which begins after the final engine ignition. As such, earlier mission stages such as the boostback burn or aerodynamic coasting are not considered in the analysis.
Therefore, the results presented here are not limited to a specific recovery strategy, since the PDL from the guidance and control perspective formulation is identical for both RTLS and DRL. The RLV under study is based on the benchmark proposed in [77]. Within the scope of the mission, only the first stage
Chair of Space Mobility and Propulsion
|
Technical University Munich
19

4. Modeling
Separation
Boostback Burn
Recovery
Burn
Thesis Reference Mission 
Separation
Recovery
Burn
Downrange landing
Return to launch site
Figure 4.1: Recovery strategies for RLVs.
is considered, with the single-engine configuration replaced by a cluster of five engines. The first stage is shown in Figure 4.2. In addition, the rocket is equipped with two pairs of steerable fins mounted near the top of the vehicle.
The analysis is restricted to the terminal portion of the propulsive landing maneuver, starting at an altitude of 5 km, with the engines assumed to be already ignited. The maximum and minimum total thrust values are evenly distributed among the five engines, obtained by dividing the total values by five.
A summary of the main vehicle parameters used in the modeling is provided in Table 4.1. The center of mass (CoM) position is measured from the gimbal point of the central engine. The reference surface area corresponds to the booster cross-section, derived from its diameter. The maximum thrust corresponds to 13.5 times the initial mass, while the minimum thrust is limited to 30% of this value. The thrust rate is obtained from the difference between these bounds, divided by 1.5.
Table 4.1: Rocket parameters
Parameter
Symbol
Value
Dry mass mdry
2750 kg
Initial mass m0
4300 kg
Specific impulse
Isp
282 s
Booster diameter d
3.0 m
Reference surface area
Sref
7.07 m2
Booster length
L
11.7 m
CoM position xCoM
4.6 m
Maximum total thrust
Tmax
58 kN
Minimum total thrust
Tmin
17.4 kN
Thrust rate limit
˙Tmax
27.1 kN/s
Maximum TVC angle
βmax y,i , βmax z,i
8◦
Maximum TVC rate
˙βmax y,i , ˙βmax z,i
4◦/s
Chair of Space Mobility and Propulsion
|
Technical University Munich
20

4. Modeling
ZB
XB
YB
Figure 4.2: Vehicle body axes and aerodynamic angles.
4.3 Equations of Motion
The equations of motion are formulated in an Up-East-North (UEN ) frame considered inertial and denoted as I, centered at the landing pad (see Figure 4.3). The validity of treating this frame as inertial will be discussed later. External forces are defined in the body–fixed frame B, whose origin is placed at the CoM, which is assumed constant during PDL, since fuel consumption in this phase is small compared to other flight phases, such as ascent. The xB axis coincides with the rocket longitudinal axis pointing toward the tip, while yB and zB complete a right–handed triad. The adopted convention is illustrated in
Figure 4.3. Detailed expressions for aerodynamic and propulsive forces are given in the following sections, for now, they are represented in compact form as a resultant force vector.
The rocket is modeled as a point mass with variable mass, moving in three–dimensional space with respect to the inertial reference frame I fixed at the landing pad. The system state is composed of the position, velocity, and mass:
r =
h x y z i⊤
∈R3
(4.1)
v =
h vx vy vz i⊤
∈R3
(4.2)
m ∈R>0
(4.3)
Here, r and v denote the rocket position and velocity expressed in the UEN frame I, whose axes are aligned with the local up, east, and north directions.
The translational dynamics are expressed as
˙r = v (4.4)
˙v = agrav + 1
m FI
(4.5)
Chair of Space Mobility and Propulsion
|
Technical University Munich
21

4. Modeling
ZI
YI
XI
XB
YB
ZB
r
Figure 4.3: Reference frames: inertial UEN frame I centered at the landing pad and body–fixed frame
B.
where agrav is the gravitational acceleration and FI is the sum of aerodynamic and propulsive forces expressed in the inertial frame.
The application of thrust leads to propellant consumption, and under the assumption of negligible backpressure losses in the engines, the corresponding mass variation is described by the classical rocket equation [86]:
˙m = −
T
g0Isp (4.6)
where T denotes the total thrust magnitude.
The gravitational acceleration is modeled using the central-body assumption:
agrav = −µ
r + rE
∥r + rE∥3
2
(4.7)
where µ = 3.986 × 1014 m3/s2 is Earth’s gravitational parameter, and rE = [RE, 0, 0]T is the position vector of the Earth’s center, with RE = 6371 km the mean Earth radius.
The orientation of the body frame with respect to the inertial frame can, under the assumption of zero bank angle, be fully described by the angle of attack and sideslip [70], illustrated in Figure 4.2. Based on this, aerodynamic and propulsive forces are expressed in the inertial frame through a Direction Cosine
Matrix (DCM), constructed as two consecutive rotations:
MIB = MIW MW B
(4.8)
where MIW and MW B represent the DCM from wind axes to inertial reference frame and from body to wind axes, respectively. The transformation MW B can be written in terms of the aerodynamic angles of
Chair of Space Mobility and Propulsion
|
Technical University Munich
22

4. Modeling attack α and sideslip β (Figure 4.2) [85]:
MW B =


cos α cos β
sin β
sin α cos β
−cos α sin β
cos β
−sin α sin β
−sin α
0
cos α


(4.9)
Once the relationship between the body axes and the wind frame is established, the second rotation from wind to inertial must be defined. As anticipated earlier, under the flat–Earth assumption the local
UEN frame can be treated as inertial, which justifies its use here.
This allows the wind–to–inertial transformation to be parameterized by the flight–path and azimuth angles commonly used in aeronautics
[70]:
γ = tan−1


−
vz q v2x + v2y


,
ψ = tan−1
vy vx

(4.10)
where vx, vy, vz are the velocity components along the north-east-down NED axes. For a rocket landing problem, however, these definitions become unsuitable: during a nearly vertical descent the flight–path angle approaches −π/2, which makes the associated rotation matrix singular. To avoid this ambiguity, following [70], the angles are redefined as the vertical flight–path angle and the vertical azimuth angle, which can be used directly to construct the wind–to–NED transformation.
In this formulation, the orientation of the wind axes is expressed without encountering the singularities of the classical definitions, and the vertical flight–path and vertical azimuth angles (Figure 4.4) are given by
γv = tan−1
vz vx

,
ψv = tan−1
 
vy p v2x + v2z
!
(4.11)
Since the inertial frame I is taken to coincide with a local UEN frame, the conversion from the standard
NED frame to UEN must also be specified. This transformation is represented by the constant matrix
MUEN,NED =


0
0
−1
0
1
0
1
0
0


(4.12)
With these definitions, the resulting DCM from wind to inertial reference frame is
MIW =


cos γv cos ψv
−sin ψv cos γv
−sin γv sin ψv cos ψv
0
sin γv cos ψv
−sin γv sin ψv cos γv


(4.13)
Chair of Space Mobility and Propulsion
|
Technical University Munich
23

4. Modeling
Figure 4.4: Vertical flight-path and azimuth angles.
Finally, by combining the gravitational model with the general representation of aerodynamic and propulsive forces, and applying the frame transformations introduced above, the equations of motion of the system can be written as













˙r = v
˙v = agrav + MIB(TB + FB)/m
˙m = −
T
g0Isp (4.14)
where TB and FB denote the propulsive and aerodynamic force vectors expressed in the body frame, respectively.
4.3.1 Aerodynamic Characteristics
The rocket model makes use of lookup tables for aerodynamic parameters, and the corresponding mapping to body forces follows the approach in [72]. The lookup tables define the axial and normal aerodynamic coefficients, as well as the x-component of the center of pressure (CoP) relative to the thrust application point of the vehicle. The aerodynamic data is organized as a two-dimensional set, and due to the axial symmetry of the booster, the coefficients extend to a three-dimensional representation.
The axial coefficient is evaluated using the Mach number and the effective angle of attack, defined as:
αeff =
p
α2 + β2
(4.15)
The axial aerodynamic coefficient is obtained from the lookup table:
CA,B = CLU
A (αeff, M)
(4.16)
Since drag acts along the velocity vector, the body-axis x-component of the aerodynamic coefficient is:
Cx,B = −CA,B
(4.17)
Chair of Space Mobility and Propulsion
|
Technical University Munich
24

4. Modeling
The y- and z-components of the aerodynamic coefficients are computed using the normal coefficient lookup table and the angles of attack and sideslip:
Cy,B = −sign(β) · CLU
N (|β|, M)
(4.18)
Cz,B = −sign(α) · CLU
N (|α|, M)
(4.19)
The aerodynamic force vector in the body frame is then given by:
FB = 1
2ρv2Sref h
Cx,B
Cy,B
Cz,B
iT
(4.20)
Using the CoP lookup table, two CoP values associated with α and β are computed:
Xα
CoP = XLU
CoP(α, M)
(4.21)
Xβ
CoP = XLU
CoP(β, M)
(4.22)
The corresponding lever arms from the CoM to each CoP are:
rα
CoM→CoP =
h
Xα
CoP
0
0
i⊤
−rCoM
(4.23)
rβ
CoM→CoP =
h
Xβ
CoP
0
0
i⊤
−rCoM
(4.24)
with rCoM =
h xCoM
0
0
i⊤
(4.25)
The torque contributions with respect to the CoM are then:
τ α
B = rα
CoM→CoP × Fα
B
(4.26)
τ β
B = rβ
CoM→CoP × Fβ
B
(4.27)
with the force components defined as:
Fα
B =
h
FB,x
0
FB,z iT
(4.28)
Fβ
B =
h
FB,x
FB,y
0
iT
(4.29)
Finally, the total torque with respect to the CoM is:
τ aero
B
= τ α
B + τ β
B
(4.30)
The atmosphere model is the U.S. Standard Atmosphere 1976 [78], which provides atmospheric density, temperature and ambient pressure as a function of altitude.
Chair of Space Mobility and Propulsion
|
Technical University Munich
25

4. Modeling
4.3.2 Propulsive Characteristics
Figure 4.5: Cross-section of the engine cluster.
The vehicle is controlled by adjusting both the magnitude and direction of the thrust vector produced by each of its five rocket engines, which all share the same thrust characteristics with identical limits on both magnitude and rate. The four lateral engines are equipped with TVC, each with two actuators that deflect the nozzle of each engine along the body-frame y- and z-axes by the angles βy and βz, respectively.
The central engine’s thrust is aligned with the body-frame x-axis, and thus does not contribute to the torque. Its force and torque expressions are given by:
TB,1(t) = T1(t) · iB
(4.31)
τ prop
B,1 (t) =
h
0
0
0
i⊤
(4.32)
where iB is the unit vector along the body-frame x-axis.
The thrust force vector of a lateral engine i ∈{2, 3, 4, 5}, with nozzle deflections βy,i(t) and βz,i(t), is given by
TB,i(t) = Ti(t)


cos
βy,i(t)

cos
βz,i(t)

cos
βy,i(t)

sin
βz,i(t)

−sin
βy,i(t)



(4.33)
and the associated torque relative to the CoM is
τ prop
B,i (t) =
h rPVP,i −rCoM
i⊤
× TB,i(t)
(4.34)
where rPVP,i is the position vector of the pivot point of engine i. The total thrust and torque in the body frame are then obtained as the sum of the contributions from all engines:
TB(t) =
5
X
i=1
TB,i(t)
(4.35)
Chair of Space Mobility and Propulsion
|
Technical University Munich
26

4. Modeling
τ prop
B
(t) =
5
X
i=1
τ prop
B,i (t)
(4.36)
4.4 Equations of Motion Summary and Assumptions
Collecting the gravitational, aerodynamic, and propulsive contributions, and applying the frame transformations introduced above, the complete equations of motion are













˙r = v
˙v = agrav + MIB(TB + FB)/m
˙m = −
P5
i=1 Ti g0Isp (4.37)
The following assumptions are made in the modeling framework:
• Only 3-DoF translational dynamics are modeled, attitude dynamics are represented implicitly through aerodynamic angles, whose values are assumed to be instantly tracked by a lower level controller.
• The CoM position is considered static.
• Flexible body effects, propellant sloshing, actuator dynamics, and structural bending modes are neglected.
• The atmosphere is represented by the U.S. Standard Atmosphere 1976 [78].
• Gravity is modeled using the central-body assumption, valid for the short flight times considered.
• Earth is considered to be flat due to limited horizontal distance that the rocket has to cover.
Chair of Space Mobility and Propulsion
|
Technical University Munich
27

5. Nominal Guidance & Control
5 Nominal Guidance & Control
5.1 Nominal Guidance
The nominal landing trajectory is obtained by solving a nonlinear OCP. The goal is to compute a feasible trajectory that drives the vehicle from the initial conditions specified in the mission profile (Section 4.2)
to a soft landing at the designated pad, located at the origin of the inertial reference frame. The OCP is constructed using the vehicle dynamics defined in Chapter 4, together with path constraints and a suitable cost function. In the following, each of these elements is introduced, after which they are collected into the formal OCP definition. The problem is subsequently scaled and solved numerically to obtain the reference trajectory.
5.1.1 State and Control Augmentation
The equations of motion used for the guidance problem are those defined in (4.37). To enforce explicit bounds on the control rates and to obtain smooth control sequences, the controls are reformulated such that the original control variables become states, while their time derivatives serve as the new control inputs in the OCP.
This augmentation introduces 15 additional first-order differential equations:
˙Ti = uTi, i = 1, . . . , 5
(5.1)
˙βy,i = uβy,i, i = 1, . . . , 4
(5.2)
˙βz,i = uβz,i, i = 1, . . . , 4
(5.3)
˙α = uα
(5.4)
˙β = uβ
(5.5)
The angle of attack during the descent phase is defined around π. For the OCP formulation, it is instead expressed in its tail formulation (see Figure 4.2), defined as
αtail = π −α
(5.6)
which shifts the variable close to zero, and the aerodynamic lookup tables are reparameterized accordingly.
In the guidance dynamics, the transformation between the body and wind frames, given by (4.9), remains unchanged and continues to use the original angle of attack by remapping αtail back to α. However, αtail is used to access the reparameterized lookup tables. This approach simplifies the OCP implementation, since α can now be constrained to values around zero. In the remainder of this work, the symbol α is
Chair of Space Mobility and Propulsion
|
Technical University Munich
29

5. Nominal Guidance & Control used for simplicity to denote the tail formulation αtail.
5.1.2 Path Constraints
A glideslope constraint is included to guide the rocket during descent by constraining the ratio of horizontal to vertical distance reduction during landing [67]:
x p y2 + z2 ≥tan γgs.
(5.7)
The glideslope angle is fixed at 45◦, and the trajectory is constrained to lie within the corresponding cone.
5.1.3 Boundary Conditions
The initial state corresponds to the conditions at the start of the powered descent phase. In particular, the mission begins at an altitude of 5 km with lateral displacements of 200 m and 300 m in the y and z directions, respectively. The terminal state enforces a soft landing at the pad, with the vertical position constrained to 1 m above ground, zero lateral displacements, zero lateral velocities, and a vertical velocity of −1 m/s to avoid a singularity in the definition of the vertical flight path angle (4.11). At landing, the aerodynamic angles and TVC deflections are required to return to zero, while they are unconstrained at the beginning. Both the final mass and final time remain free. All boundary conditions and box constraints are summarized in Table 5.2.
5.1.4 Objective Function
The objective is to minimize the integral of a stage cost composed of several weighted terms:
ℓ(x, u) = wT JT + w ˙T J ˙T + w ˙βy ˙βz

J ˙βy + J ˙βz

+ w ˙α ˙β

J ˙α + J ˙β

+ wprop
τ
Jτ prop + waero
τ
Jτ aero.
(5.8)
Each component in (5.8) serves a specific purpose in shaping the solution, and the corresponding weights are listed in Table 5.1.
Table 5.1: NLP integral weights
Weight
Value wT
1
w ˙T
10−3
w ˙βy ˙βz
5 · 10−1
w ˙α ˙β
6
wprop
τ
10−2
waero
τ
10−4
Thrust magnitude:
A quadratic penalty on the thrust magnitude leads the optimizer to avoid unnecessarily high thrust levels. Since propellant consumption is directly related to thrust through the mass
Chair of Space Mobility and Propulsion
|
Technical University Munich
30

5. Nominal Guidance & Control depletion equation (4.6), this term effectively promotes fuel efficiency.
JT =
5
X
i=1
T 2
i .
(5.9)
Control rates:
The sum of squares of the control rates promotes smooth control trajectories:
J ˙T =
5
X
i=1
˙T 2
i (5.10)
J ˙βy =
4
X
i=1
˙β2
y,i,
J ˙βz =
4
X
i=1
˙β2
z,i (5.11)
J ˙α = ˙α2,
J ˙β = ˙β2
(5.12)
Torques:
Finally, quadratic penalties are imposed on the torques generated by the propulsive and aerodynamic forces:
Jτ prop =
3
X
i=1
τ prop
B
(i)2
(5.13)
Jτ aero =
3
X
i=1
τ aero
B
(i)2
(5.14)
The inclusion of the propulsive and aerodynamic torque penalties in the cost function is motivated by the fact that, despite a 3-DoF formulation, we want to ensure that the resulting solution remains compatible with a full six-degree-of-freedom (6-DoF) model.
In other words, the computed trajectory must be trimmable.
Table 5.2: NLP box constraints and boundary conditions
Variable
Units
Lower Bound
Upper Bound
IC
FC
t s
5
100
0
–
x m
0
5000
5000
1
y m
−5000
5000
100
0
z m
−5000
5000
−200
0
vx m/s
−500
500
−219.43
−1
vy m/s
−0.1
500
−13.17
0
vz m/s
−500
500
−8.78
0
m kg mdry m0
m0
–
Ti kN
1.1 Tmin,i
0.9 Tmax,i
–
–
βy,i deg
−5
5
–
0
βz,i deg
−5
5
–
0
α
deg
−10
10
–
0
β
deg
−10
10
–
0
˙Ti kN/s
−˙Tmax
˙Tmax
–
–
˙βy,i deg/s
−˙βmax y,i
˙βmax y,i
–
0
˙βz,i deg/s
−˙βmax z,i
˙βmax z,i
–
0
˙α, ˙β
deg/s
−5
5
–
0
Integral
–
0
100
–
–
Chair of Space Mobility and Propulsion
|
Technical University Munich
31

5. Nominal Guidance & Control
5.1.5 Problem Statement
Collecting the elements defined above yields the following OCP:
Problem 1: Continuous-Time Free-Final-Time Non-Convex Problem
Minimize over x, u, tf
J =
Z tf
0
ℓ(x, u)dτ
Subject to Dynamics:
˙r = v
˙v = agrav + MIB
TB + FB

/m
˙m = −
P5
i=1 Ti g0Isp
Path Constraints:
x p y2 + z2 ≥tan γgs
Box Constraints:
x(t) ∈[xmin, xmax]
u(t) ∈[umin, umax]
Boundary Conditions:
r(t0) = r0
v(t0) = v0
m(t0) = m0
r(tf) = rf v(tf) = vf
α(tf) = 0
β(tf) = 0
βy,i(tf) = 0
βz,i(tf) = 0
i = 1, . . . , 4
5.1.6 Initialization
For the initial guess, the position and velocity states are obtained by linear interpolation between the boundary conditions, while the mass is initialized by linear interpolation between the initial and dry mass. The thrust states are also initialized by linear interpolation between their minimum and maximum bounds, and all remaining variables are set to zero when no prior information is available.
5.1.7 Scaling
One important aspect that strongly influences the performance of an optimization algorithm is the scaling of the problem. A problem is considered poorly scaled if changes in one optimization variable lead to significantly larger variations in the cost function than those caused by variations in another variable
[55]. In such cases, the numerical optimizer may struggle, leading to slow convergence or even failure to converge. In comparison, a properly scaled NLP can be solved much more efficiently.
Scaling issues typically arise when the relative magnitudes of variables or constraints differ significantly, or when their values deviate significantly from unity. A standard approach is to apply a transformation
Chair of Space Mobility and Propulsion
|
Technical University Munich
32

5. Nominal Guidance & Control of units so that optimization variables lie approximately within the range [−1, 1].
Four fundamental scaling quantities are typically selected: length, speed, time, and mass. All other scales are derived from these [60]. In this work, the length, speed, and mass scales are set to their maximum values, while scales for other variables are obtained by combining these fundamental quantities. The adopted scaling is summarized in Table 5.3.
Table 5.3: Scaling of fundamental and derived quantities
Quantity
Symbol
Definition
Value
Length
L
-
5000 m
Speed
V
-
500 m/s
Time
T
L/V
10 s
Mass
M
-
4300 kg
Acceleration a
V/(L/V )
50 m/s2
Area
A
L2
2.5 × 107 m2
Volume
Vol
L3
1.25 × 1011 m3
Density
ρ
M/Vol
3.44 × 10−8 kg/m3
Thrust
F
MV/(L/V )
2.1 × 105 N
Once the scaling factors are defined, all bounds, initial guesses, and problem parameters are expressed in the scaled domain, which allows the NLP solver to handle variables of comparable magnitudes and thereby improves convergence.
5.1.8 Nominal Trajectory Solution
The OCP defined in Problem 1 is solved using GPOPS-II [58], a MATLAB package that implements variable-order Gaussian quadrature collocation methods, transforming the continuous-time OCP into a sparse NLP, which is then solved using IPOPT [90]. Although GPOPS-II provides automatic scaling options, manual scaling is often preferred as it ensures better conditioning of the problem.
Figure 5.1a shows the position and velocity profiles, while Figure 5.1b illustrates the mass consumption.
Figures 5.2a and 5.2b show the propulsive and aerodynamic augmented states, which correspond to the actual control inputs to the system as the augmentation introduced in Section 5.1.1 is applied solely for numerical reasons and does not alter the physical interpretation of the controls. The vehicle lands with approximately 970 kg of fuel remaining, corresponding to about 1% of the total propellant mass, which is consistent with the 0.7% margin reported in the benchmark [77].
5.1.9 Solution Verification
Optimality Check
One of the defining properties of the Gauss pseudospectral method is that it enables the Karush–Kuhn–Tucker (KKT) of the NLP to be mapped to the costates of the continuous-time OCP [13]. Therefore, the optimality of the solution can be verified a posteriori via the same procedure as in [70]. The costate vector is defined as
λ =
h
λT
r
λT
v
λm
λT
T
λT
βy
λT
βz
λα
λβ
iT
∈R22
(5.15)
Chair of Space Mobility and Propulsion
|
Technical University Munich
33

5. Nominal Guidance & Control
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
(a) Position and velocity.
0
10
20
30
40
50
60
3700
3800
3900
4000
4100
4200
4300
(b) Mass.
Figure 5.1: GPOPS states: position, velocity, and mass.
Chair of Space Mobility and Propulsion
|
Technical University Munich
34

5. Nominal Guidance & Control
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
(a) Propulsive controls.
0
10
20
30
40
50
60
-2.5
-2
-1.5
-1
-0.5
0
0
10
20
30
40
50
60
0
1
2
3
4
(b) Aerodynamic controls.
Figure 5.2: GPOPS controls.
Chair of Space Mobility and Propulsion
|
Technical University Munich
35

5. Nominal Guidance & Control and the Hamiltonian can be formulated as
H = λT
r v + λT
v fv(x, u) −λm
P5
i=1 Ti
Ispg0
+
5
X
i=1
λT,i ˙Ti +
4
X
i=1
λβy,i ˙βy,i+
+
4
X
i=1
λβz,i ˙βz,i + λα ˙α + λβ ˙β + ℓ(x, u)
(5.16)
with fv(x, u) given by (4.5).
Since the Hamiltonian does not depend explicitly on time, it remains constant along the optimal trajectory. Moreover, as this is a free final time problem, the transversality condition [21] requires
H(tf) = 0
(5.17)
Hence, the Hamiltonian along the optimal trajectory satisfies
H(t) = 0, t ∈[t0, tf]
(5.18)
The optimality check therefore consists of reconstructing the Hamiltonian using the costates returned by
GPOPS-II and verifying that it remains close to zero. As these costates are approximations of those of the original OCP, this procedure provides an a posteriori validation of the computed solution. Figure 5.3
shows the reconstructed Hamiltonian, which remains close to zero throughout the trajectory, thereby confirming the optimality of the solution.
0
10
20
30
40
50
60
-2
-1
0
1
2
3
4
5
6
10-3
Figure 5.3: Optimality check: reconstructed Hamiltonian.
Chair of Space Mobility and Propulsion
|
Technical University Munich
36

5. Nominal Guidance & Control
Feasibility Check
An a posteriori feasibility check of the solution consists in verifying that the state trajectories obtained through a Runge–Kutta 45 propagation (ode45 in MATLAB, implementing the Dormand–Prince method
[27, 75]) of the initial conditions, using the interpolated controls from the numerical solution of the OCP, remain within a specified accuracy of those computed by the numerical optimization [43]. This validation confirms that the optimized trajectory satisfies the full nonlinear dynamics, with errors on the order of
2 m in position, as shown in Figure 5.5. Figure 5.4 compares the Runge–Kutta propagation with the
GPOPS-II solution for the position and velocity states.
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
Figure 5.4: Validation of GPOPS solutions: discrepancy between Runge-Kutta and GPOPS states.
5.2 Nominal Controller
The trajectory optimization described in the previous section provides an open-loop guidance solution, i.e., a sequence of states and control inputs that satisfy the dynamics. However, in practice such an open-loop approach cannot be followed accurately due to modeling limitations, parameter uncertainties, and external disturbances. A feedback controller is therefore required to track the reference trajectory and ensure robust performance under these nonideal conditions, as illustrated in the block diagram of
Figure 5.6. The design of the controllers follows a methodical procedure [68], summarized below:
1. Select the number of operating points at which controllers will be synthesized.
2. Identify a parameter that changes monotonically along the trajectory to serve as the scheduling variable. In this work, time is chosen.
3. Sample the states and control inputs of the reference trajectory at discrete points along the scheduling parameter.
Chair of Space Mobility and Propulsion
|
Technical University Munich
37

5. Nominal Guidance & Control
0
10
20
30
40
50
60
0
0.5
1
1.5
2
2.5
Figure 5.5: Validation of GPOPS solutions: position error over time.
4. Linearize the nonlinear equations of motion numerically at each sampled point, yielding a family of linear time-invariant (LTI) systems.
5. Design and analyze controllers for each of these LTI models.
6. Evaluate the individual controllers in both frequency and time domains.
7. Implement a gain-scheduled controller by interpolating between the gain matrices according to the current time.
8. Assess the closed-loop performance of the overall nonlinear system under the scheduled control law.
In this work, 13 operating points are distributed along the reference trajectory, at which a Linear
Quadratic Regulator (LQR) [6] is synthesized. LQR is adopted due to its simplicity, as the primary focus of this thesis lies on the guidance aspects. Its objective is to track the nominal trajectory by minimizing a quadratic performance index that penalizes deviations in the states as well as the control effort.
The weighting matrices Q and R determine the trade-off between tracking performance and actuation usage. The resulting feedback gains are applied in a state-feedback fashion, where the control action depends on the deviation from the reference trajectory. The controller consists of a single loop regulating translational motion by commanding thrust magnitudes, TVC deflections, and aerodynamic angles based on errors in position and velocity, while actuator saturation limits are imposed separately on each control channel as LQR does not explicitly account for constraints.
The controller design is based on a reduced-order system, obtained by eliminating the mass row and column from the A matrix and the mass row from the B matrix of each LTI model derived in Step 4. This
Chair of Space Mobility and Propulsion
|
Technical University Munich
38

5. Nominal Guidance & Control
Nominal guidance
Nominal LQR
PLANT
+
-
+
-
Figure 5.6: Block diagram: nominal closed-loop.
reduction is motivated by the fact that vehicle mass cannot be directly measured or influenced through feedback, and its inclusion in the control law would therefore be meaningless. From this procedure, 13
gain matrices of dimension 15 × 6 are obtained, leading to the following control law:
u = uref −KLQR (x −xref), (5.19)
where the reference control vector is uref =
h
T1, T2, T3, T4, T5, βy,1, βy,2, βy,3, βy,4, βz,1, βz,2, βz,3, βz,4, α, β
i⊤
, (5.20)
and the state vector is x =
h rT
vT
i⊤
, xref =
h rT
ref vT
ref i⊤
.
(5.21)
5.2.1 Linear Analysis of the Closed-Loop System
The sensitivity and complementary sensitivity transfer functions of the closed-loop system, consisting of the controller in feedback with the plant, are considered.
Frequency Domain Analysis
The Nichols chart is a standard tool for assessing the stability and robustness of a control system, as it allows the gain and phase margins to be read directly [36]. These margins quantify how close the system operates to instability, which occurs when the gain reaches 0 dB or higher while the phase simultaneously equals −180◦. By providing a visual representation of this relationship, the Nichols chart offers a convenient way to evaluate robustness. Figure 5.7 presents the Nichols chart for the synthesized controllers. No instabilities are observed, and, because the system dynamics resemble those of a double integrator, the gain margins for all input channels are infinite.
The output sensitivity transfer function So indicates good disturbance rejection at low frequencies, with no significant peaks present (Figures 5.8a and 5.8b).
Chair of Space Mobility and Propulsion
|
Technical University Munich
39

5. Nominal Guidance & Control
-250
-200
-150
-100
-50
0
-200
-100
0
100
0
0.2
0.4
0.6
0.8
1
-250
-200
-150
-100
-50
0
-200
-100
0
100
0
0.2
0.4
0.6
0.8
1
-250
-200
-150
-100
-50
0
-200
-100
0
100
0
0.2
0.4
0.6
0.8
1
-250
-200
-150
-100
-50
0
-200
-100
0
100
0
0.2
0.4
0.6
0.8
1
Figure 5.7: Closed-loop stability analysis: Nichols charts.
Time Domain Analysis
Step responses of the position and velocity outputs are computed to evaluate the closed-loop behavior in the time domain. For command tracking, the steady-state values are expected to converge to unity, whereas for disturbance rejection they should remain close to zero. Figures 5.9a and 5.9b confirm these expectations, showing accurate tracking performance in both position and velocity for the vertical and lateral channels.
5.2.2 Nonlinear Analysis
The controller is evaluated in a simulated closed-loop environment on a plant that evolves according to (4.37) and accounts for uncertainties in the initial conditions and aerodynamic parameters, which are summarized in Table 5.4, with N denoting normally distributed variables and U denoting uniformly distributed variables.
Uncertainties in the initial conditions are summed to the nominal ones while perturbations in air density and aerodynamic coefficients are introduced multiplicatively:
ρ = (1 + ∆ρ) ρnom (5.22)
Cx = (1 + ∆Cx) Cx,nom,
Cy = (1 + ∆Cy) Cy,nom,
Cz = (1 + ∆Cz) Cz,nom (5.23)
Chair of Space Mobility and Propulsion
|
Technical University Munich
40

5. Nominal Guidance & Control
10-6
100
106
-150
-100
0
20
0
10
20
30
40
50
10-6
100
106
-150
-100
0
20
0
10
20
30
40
50
(a) Output sensitivity for x-position.
10-6
100
106
-150
-100
0
20
0
0.5
1
10-6
100
106
-150
-100
0
20
0
0.5
1
10-6
100
106
-150
-100
0
20
0
0.5
1
(b) Complementary sensitivity for position channels.
Figure 5.8: Frequency domain analysis.
Chair of Space Mobility and Propulsion
|
Technical University Munich
41

5. Nominal Guidance & Control
0
5
10
15
20
25
30
35
40
45
50
0
0.2
0.4
0.6
0.8
1
0
10
20
30
40
50
0
5
10
15
20
25
30
35
40
45
50
0
0.05
0.1
0.15
0.2
0
10
20
30
40
50
(a) Step response for x-position.
0
10
20
30
40
50
60
0
0.2
0.4
0.6
0.8
1
0
10
20
30
40
50
0
10
20
30
40
50
60
-0.01
-0.005
0
0.005
0.01
0
10
20
30
40
50
(b) Step response for y-position.
Figure 5.9: Time domain analysis: step responses for x- and y-position.
Chair of Space Mobility and Propulsion
|
Technical University Munich
42

5. Nominal Guidance & Control
Table 5.4: Initial conditions and parameter uncertainties
Variable
Distribution
Units
∆ry,z
N(0, 100)
m
∆vx,y,z
N(0, 20)
m/s
∆m
U(−2%, 2%)
kg
∆ρ
N(0, 10%)
kg/m3
∆Cx,y,z
N(0, 10%)
-
To evaluate the performance and robustness of the controller, a Monte Carlo campaign of 1000 cases is conducted with uncertainties sampled according to Table 5.4. A simulation is considered successful if the landing requirements listed in Table 5.5 are satisfied.
Figure 5.10 illustrates the results, showing adequate trajectory tracking under uncertainties, with approximately 70% of the cases resulting in a successful landing.
Table 5.5: Successful landing requirements
Requirement
Value
Vertical velocity at touchdown vx(tf) ≥−2.5 m/s
Lateral velocities at touchdown
|vy(tf)|, |vz(tf)| ≤1.5 m/s
Lateral position error at touchdown
|y(tf)|, |z(tf)| ≤10 m
Figure 5.10: Closed-loop Monte Carlo campaign: successful landings.
In conclusion, the nominal trajectory provides the baseline against which the solutions of the following chapter are compared, while the gain-scheduled LQR controller is included to enable closed-loop simulations presented in Chapter 7.
Chair of Space Mobility and Propulsion
|
Technical University Munich
43

6. Online Guidance
6 Online Guidance
6.1 Sequential Convex Programming: Theoretical Overview
Sequential Convex Programming (SCP) is an iterative approach that tackles nonconvex optimization problems by repeatedly solving convex subproblems. The key idea is to retain the convex parts and replace nonconvex components with convex approximations [28].
Consider a general nonlinear optimization problem of the form min x f(x)
subject to gi(x) ≤0, i = 1, . . . , m hj(x) = 0, j = 1, . . . , p (6.1)
In this formulation, the objective function f(x) and the inequality constraints gi(x) can be nonconvex, and the equality constraints hj(x) are generally non-affine, making the resulting optimization problem non-convex. SCP solves this problem iteratively by constructing an approximation of the solution x(k), while confining the optimization problem to a convex trust region around it [28]. Within this trust region:
• The nonconvex functions f(x), gi(x) are replaced by convex approximations ˆf(x), ˆgi(x) within the trust region P(k).
• The nonlinear equality constraints hj(x) are replaced by affine approximations ˆhj(x) within the trust region P(k).
These approximations define the convex optimization subproblem:
min x
ˆf(x)
s.t.
ˆgi(x) ≤0, i = 1, . . . , m
ˆhj(x) = 0, j = 1, . . . , p x ∈P(k)
(6.2)
with the trust region defined as
P(k) =
n x ∈Rn  ∥x −x(k)∥2 ≤ρ(k) o
.
(6.3)
While affine approximations obtained through linearization are the most common approach to handle nonlinear equality constraints, linearization about the current estimate does not always guarantee convergence of the iterative scheme. To address the shortcomings of pure linearization, the convex–concave decomposition (CCD) method has been shown to perform well for certain classes of problems with nonChair of Space Mobility and Propulsion
|
Technical University Munich
45

6. Online Guidance linear equality constraints [48], however, for other types of problems it may suffer from infeasibility, particularly in the early SCP iterations. To overcome this limitation, [73] introduced the augmented convex–concave decomposition (ACCD), which guarantees feasibility from the very first iteration.
Although these methods represent important developments, they are outside the scope of this thesis, and the present work adopts the standard SCP framework instead.
6.2 Algorithm Derivation
This section derives the SCP algorithm step by step from the nonconvex OCP introduced in Chapter 5
and summarized in Problem 1, and Figure 6.1 provides an overview of the procedure.
Linearize nonconvexities
Address artificial infeasibility and unboundedness
Discretize continuous time problem
Initial
Guess
Solve convex subproblem
Convex
Solver:
-Mosek
CVX
transcription true
Convergence
Check
True
False
Iteration
Convex Subproblem Creation
STOP
START
Figure 6.1: SCP algorithm schematic.
Problem 1 is a nonconvex optimal control problem because of nonlinearities in the system dynamics, and cost function. To understand which parts make the problem nonconvex, we analyze each component separately, separating convex and nonconvex terms. The analysis proceeds in three parts: the system dynamics, the cost function, and the path constraints. Finally, the formulation is extended to account for the free–final–time nature of the problem.
The system dynamics can be equivalently written by explicitly separating the convex and nonconvex terms:
˙x = fnc(x) + fc(x) + Bu (6.4)
where the convex terms are represented by fc(x) =


v
03×1
−
P5
i=1 Ti
Ispg0
015×1


= Acx (6.5)
Chair of Space Mobility and Propulsion
|
Technical University Munich
46

6. Online Guidance with
Ac =


03×3
I3×3
03×16
03×3
03×3
03×16
01×7
−
1
Ispg0 11×5
01×10
015×3
015×3
015×16


(6.6)
The control input matrix is defined as
B =

07×15
I15×15


(6.7)
Since Ac and B only contain constant terms, they need to be computed only once.
In Chapter 5, we motivated the choice of augmented states and control rates as inputs by noting that this formulation allows us to impose explicit bounds on the rates and to obtain smooth control sequences.
In SCP, an additional benefit of using a system that is affine in the controls is that there is no need to compute Jacobians with respect to the control variables. According to [46], linearization with respect to the control variables introduces high-frequency chatter in the control profile due to numerical instability, which is generally detrimental to the convergence of the successive solution process. Small oscillations in un−1 propagate into the elements of the linearized dynamics, and as a result the updated control un is likely to amplify these chattering effects.
The nonconvex terms are therefore limited to the velocity states fnc(x) =


03×1
agrav + aprop + aaero
015×1


(6.8)
and finally, the overall dynamics can be expressed as
˙x = f(x, u) = Acx + fnc(x) + Bu (6.9)
The glideslope constraint (5.7) is conic by nature and therefore does not require convexification, it can in fact be expressed as SOC constraint [66].
 

y, z

, x tan γgs
!
∈K3
(6.10)
which, as discussed in Section 3.1.3, is equivalent to

h y z i⊤
2
≤
x tan γgs (6.11)
The box constraints are linear inequalities and therefore fully compatible with convex optimization.
The remaining source of nonconvexity arises from the objective function, which, as with the dynamics, can be separated into convex and nonconvex terms:
ℓ(x, u) = ℓc(x, u) + ℓnc(x, u)
(6.12)
Chair of Space Mobility and Propulsion
|
Technical University Munich
47

6. Online Guidance
The convex term includes all sums of squares of states and controls (5.9)–(5.12) and since each term is a convex function, their sum preserves convexity (3.12).
ℓc(x, u) = wT JT + w ˙T J ˙T + w ˙βy ˙βz

J ˙βy + J ˙βz

+ w ˙α ˙β

J ˙α + J ˙β

(6.13)
On the other hand, the propulsive and aerodynamic torques are formulated through nonlinear expressions of the optimization variables, thus their relative cost terms represent another source of nonconvexity.
ℓnc(x, u) = ℓτ(x, u) = wprop
τ
Jprop
τ
+ waero
τ
Jaero
τ
(6.14)
Lastly, to account for the free final time in the problem, the dynamics are expressed in terms of a normalized trajectory time τ ∈[0, 1], following the approach in [87]. The original time derivatives can then be related to the normalized time via the chain rule:
dx(t)
dt
= dτ
dt · dx dτ
(6.15)
Introducing the time–dilation coefficient σ, which maps τ to the actual time t, gives
σ :=
dτ
dt
−1
= dt dτ = tf −t0 = tf (6.16)
With this definition, the dynamics in normalized time become x′(τ) = σ · f(x(τ), u(τ))
(6.17)
where the time–dilation coefficient σ is treated as an optimization variable.
6.2.1 Convexification Procedure
To construct a convex subproblem, we begin by eliminating all nonconvex components of the original problem. This is achieved by replacing each nonconvex term with its first-order approximation around a reference denoted as {˜x(τ), ˜u(τ), ˜σ}, quantities that will be explicitly defined in Section 6.2.2. Through the separation of convex and non-convex terms, we can apply a partial linearization and apply sequential convex programming by leveraging the difference between convex and non-convex terms.
Therefore, the normalized time dynamics (6.17) can be rewritten as:
˙x(τ) = f(˜x(τ), ˜u(τ)) σ + A(τ) x(τ) + Bσ u(τ) + z(τ)
(6.18)
with
A(τ) =
Anc(τ) + Ac

˜σ
(6.19)
Bσ = B ˜σ
(6.20)
and
Anc(τ) = ∂fnc(x, u)
∂x

˜x(τ), ˜u(τ)
(6.21)
Chair of Space Mobility and Propulsion
|
Technical University Munich
48

6. Online Guidance z(τ) = −A(τ) ˜x(τ) −Bσ(τ) ˜u(τ)
(6.22)
Because of linearization, the resulting subproblem may become infeasible, even if the original nonconvex problem is feasible. This phenomenon, known as artificial infeasibility [49], typically occurs when the intersection of the convexified constraints is empty, which would prematurely terminate the algorithm.
Artificial infeasibility is most common in the early iterations, often caused by a poor initial guess. For instance, if the initial estimate of the final time is far from a realistic value, the linearized dynamics may fail to admit any feasible solution.
A common remedy is to relax the linearized dynamics equality constraints by introducing slack variables, known as a virtual controls, which are left unconstrained but penalized heavily in the cost function [87].
ν(τ) ∈R22
(6.23)
With this modification, the linearized dynamics become
˙x(τ) = f(˜x(τ), ˜u(τ)) σ + A(τ) x(τ) + Bσ u(τ) + z(τ) + Cν
(6.24)
where the matrix C is a parameter that determines which virtual controls are introduced to assist the convergence process. In our case, we set
C = I22
(6.25)
which implies that the virtual controls are applied to all the states. Intuitively, the virtual controls serve as auxiliary inputs that are only activated when necessary to prevent infeasibility. For the final solution to be feasible and physically consistent with the original problem, the virtual control terms must vanish.
Following [87], each virtual control vector is bounded by a corresponding slack variable κ:
∥ν(τ)∥2 ≤κ(τ)
(6.26)
and to progressively eliminate them during the optimization, an additional slack variable is used to bound the norm of κ:
∥κ∥2 ≤sκ, sκ ∈R
(6.27)
where sκ is included in the cost function and weighted according to its corresponding penalty term.
The remaining nonconvex terms, specifically the torques in the cost function, can also be expressed in a convex form through linearization around the reference. The integrand is convexified by expanding the nonlinear torque terms to first order:
ℓconvex(x, u) = ℓc(x, u) + ℓτ(˜x, ˜u) + ∂ℓτ
∂x

˜x,˜u (x −˜x) + ∂ℓτ
∂u

˜x,˜u (u −˜u)
(6.28)
Since the torques only depend on the augmented states, the final integral expression becomes
ℓconvex(x, u) = ℓc(x, u) + (Aprop
τ
+ Aaero
τ
)(x −ˆx)
(6.29)
with
Aprop
τ
= ∂τ prop
B
(x)
∂x

˜x
,
τ prop
B
(x) =
h
03×8
∗
03×2
i (6.30)
Chair of Space Mobility and Propulsion
|
Technical University Munich
49

6. Online Guidance where the nonzero block ∗corresponding to the columns from 9 to 20, and
Aaero
τ
= ∂τ aero
B
(x)
∂x

˜x
,
τ aero
B
(x) =
h
03×20
∗
i (6.31)
where the nonzero block ∗corresponding to the columns from 21 to 22.
Linear approximations are only accurate in the vicinity of the trajectory about which the system is linearized. For this reason, the subproblem solution must be kept reasonably close to the linearization point characterized by the reference solution, which in the case of SCP, is the solution at the previous iteration. This motivation leads to the use of trust region constraints. Different formulations have been proposed in the literature. One option is to prescribe a constant, user-specified trust region radius [44].
This approach decreases the optimization subproblem size since the radius is treated as fixed input rather than an optimization variable. Another strategy is to define update rules for the trust region radius, expanding or contracting it according to measures of how well the linear model captures the nonlinear dynamics [50, 92]. This approach comes with the benefit of still keeping the optimization problem size contained, as the radius is constant with respect to the optimization problem that is being solved, while taking into account other information and updating its value between iterations. A further alternative is to embed the trust region bounds directly into the optimization problem itself [67, 88].
Adopting the latter idea, we constrain the change between consecutive solutions by augmenting the problem with a slack variable that limits the maximum allowable deviation. In particular, we require
X(τ) −˜X(τ)
 ≤η(τ)
(6.32)
where X(τ) = [x(τ), u(τ)] collects states and controls, ˆX(τ) = [˜x(τ), ˜u(τ)] denotes the corresponding values at the reference about which the linearization occurs, and η(τ) specifies the allowable deviation, which is penalized in the cost through a corresponding slack variable:
∥η∥2 ≤sη, sη ∈R
(6.33)
Lastly, the time-dilation coefficient is bounded through a fixed size trust region.
∥σ −˜σ∥≤ησ = const.
(6.34)
In order to obtain a fully linear cost function, as required for the SOCP formulation, the integral term is bounded by an additional slack variable sR , which is then penalized in the cost through a weighting factor wR :
Z 1
0
ℓconvex(τ) dτ ≤sR
(6.35)
The augmented cost function then becomes
J = wR sR + wη sη + wκ sκ + wσ σ
(6.36)
where the last term penalizes the time–dilation coefficient σ, whose role will be clarified later.
With these modifications, the formulation results in a free-final-time, continuous-time convex optimal control problem, summarized as Problem 2.
Chair of Space Mobility and Propulsion
|
Technical University Munich
50

6. Online Guidance
Problem 2: Continuous-Time Fixed-Final-Time Convex Problem
Minimize over x, u, ν, η, κ, sη, sκ, σ
J = wR sR + wη · sη + wκ · sκ + wσ · σ
Subject to Linearized Dynamics:
˙x(τ) = f(ˆx(τ), ˆu(τ)) σ + A(τ) x(τ) + Bσ u(τ) + z(τ) + Cν
Path Constraints:
 

y, z

, x tan γgs
!
∈K3
State and Control constraints:
x(τ) ∈[xmin, xmax]
u(t) ∈[umin, umax]
Virtual Control and Trust Region Constraints:
∥ν(τ)∥2 ≤κ(τ)
∥κ∥2 ≤sκ
X(τ) −˜X(τ)
 ≤η(τ)
∥η∥2 ≤sη
∥σ −˜σ∥≤ησ
Z 1
0
ℓconvex(τ) dτ ≤sR
Boundary conditions:
r(0) = r0, v(0) = v0
m(0) = m0
r(1) = rf v(1) = vf
α(1) = 0
β(1) = 0
βy,i(1) = 0
βz,i(1) = 0
i = 1, . . . , 4
6.2.2 Discretization
To solve the continuous-time optimal control problem, we transcribe it into a finite-dimensional parameter optimization problem. This is achieved through temporal discretization, carried out using the trapezoidal scheme. The normalized time variable τ ∈[0, 1] is partitioned uniformly into K −1 intervals with K
Chair of Space Mobility and Propulsion
|
Technical University Munich
51

6. Online Guidance nodes {τ1, τ2, . . . , τK−1, τK}, where τ1 = 0, τK = 1. The step size is therefore
∆τ =
1
K −1.
(6.37)
and normalized time at node k is
τk = k −1
K −1, k = 1, 2, . . . , K
(6.38)
Here, the subscript k denotes the time step, while the superscript n indicates the iteration number. The discretized optimal control problem at iteration n is defined by the following dynamics:
xk+1 = xk + ∆τ
2
h f n−1
k
σ + An−1
k xk + Bσuk + zn−1
k
+ Cνk
+ f n−1
k+1 σ + An−1
k+1xk+1 + Bσuk+1 + zn−1
k+1 + Cνk+1
i
, k = 1, . . . , K −1
(6.39)
with f n−1
k
= f(xn−1
k
, un−1
k
)
(6.40)
An−1
k
=
An−1
nc,k + Ac

σn−1
(6.41)
Bσ = B σn−1
(6.42)
An−1
nc,k = ∂fnc(x)
∂x

xn−1
(6.43)
zn−1
k
= −An−1
k xn−1
k
−Bσun−1
k (6.44)
The continuous-time glideslope constraint in (6.10) is enforced at each discretization node through K
SOC constraints:
 
[yk, zk], xk tan γgs
!
∈K3
(6.45)
As highlighted in Rao’s survey [60], the quadrature used for the Lagrange term, that is, the integral term in the cost function, should match the numerical integration scheme adopted for the dynamics.
Accordingly, the trapezoidal rule is also used to approximate the integral term.
K−1
X
k=1
∆τ
2
h
ℓn convex,k(xk, uk) + ℓn convex,k+1(xk+1, uk+1)
i
≤sR
(6.46)
with
ℓn convex,k(x, u) = ℓc(xk, uk) + τ B(xn−1
k
)σ + An−1
τ,k xk + zn−1
τ,k (6.47)
An−1
τ,k =
An−1,prop
τ,k
+ An−1,aero
τ,k

σn−1
(6.48)
zn−1
τ,k = −An−1
τ,k xn−1
k (6.49)
An−1,prop
τ,k
= ∂τ prop
B
(x)
∂x

xn−1
k (6.50)
Chair of Space Mobility and Propulsion
|
Technical University Munich
52

6. Online Guidance
An−1,aero
τ,k
= ∂τ aero
B
(x)
∂x

xn−1
k (6.51)
The virtual control vector at time step k, denoted by νk ∈R22, is bounded by the corresponding slack variable κk, where κ = [κ1, . . . , κK]⊤∈RK. The vector κ is itself bounded by the slack variable sκ ∈R:
∥νk∥2 ≤κk (6.52)
∥κ∥2 ≤sκ
(6.53)
Similarly, the trust region variable ηk at time step k bounds the deviation between the current optimization variables and their values at the previous iteration n −1. Collecting all ηk into the vector η ∈RK, we impose the following bounds:
∥xk −xn−1
k
∥2
2 + ∥uk −un−1
k
∥2
2 ≤ηk (6.54)
∥η∥2 ≤sη
(6.55)
The complete set of optimization variables, together with their dimensions, is summarized in Table 6.1, whereas Problem 3 presents the final discrete-time, fixed-final-time convex formulation, and 6.2.2 provides a high-level description of the SCP algorithm.
SCP Pseudocode
Input: Select weights wη, wκ, wR , wσ, tolerances ϵη, ϵν, maximum iterations Nmax, and initial guess z0 = {x0, u0, σ0}.
for n ∈{2, . . . , Nmax} do:
1. Linearize torques about zn−1 = {xn−1, un−1, σn−1} →An−1
τ
, zn−1
τ
using (6.48)–(6.51)
2. Linearize dynamics about zn−1 →An−1, f n−1, zn−1 using (6.40)–(6.44)
3. Solve Problem (3) →zn = {xn, un, σn}
4. if (6.56) & (6.57):
STOP
5. else:
continue with the next iteration
Chair of Space Mobility and Propulsion
|
Technical University Munich
53

6. Online Guidance
6.2.3 Convergence Check
After each convex subproblem is solved, convergence is verified. If the criteria are satisfied, the algorithm terminates and the current solution is accepted. Convergence is declared when both of the following conditions hold:
sκ ≤ϵν
(6.56)
sη ≤ϵη
(6.57)
Condition (6.56) ensures that the virtual controls vanish, reflecting the fact that they are introduced only to aid convergence and to prevent infeasibility in the early iterations. Condition (6.57) requires the solution to remain sufficiently close to the one obtained at the previous iteration, which guarantees that the update lies within the region where the linear approximation is valid.
Table 6.1: SCP Optimization variables
Variable
Dimension
Description x
[22 × K]
State trajectory u
[15 × K]
Control trajectory
σ
[1]
Time-dilation coefficient
ν
[22 × K]
Virtual controls
η
[K]
Trust region size
κ
[K]
Virtual controls vector slack sκ
[1]
Virtual controls slack sη
[1]
Trust region slack sR
[1]
Integral slack
6.2.4 Initialization
The initial guess for the position and velocity states is obtained by linearly interpolating between the initial and final boundary conditions. The thrust magnitudes are initialized and held constant at their minimum value, while the TVC and aerodynamic angles are set to zero. All control rates are also initialized to zero.
This simple initialization scheme is chosen to validate the performance of the algorithm by comparing it with the results obtained with GPOPS-II in Chapter 5.
Chair of Space Mobility and Propulsion
|
Technical University Munich
54

6. Online Guidance
Problem 3: Discrete-Time Fixed-Final-Time Convex Problem
Minimize over x, u, ν, κ, η, sη, sκ, sR , σ
J = wR sR + wη · sη + wκ · sκ + wσ · σ
Subject to linearized dynamics:
xk+1 = xk + ∆τ
2
h f n−1
k
σ + An−1
k xk + Bσuk + zn−1
k
+ Cνk
+ f n−1
k+1 σ + An−1
k+1xk+1 + Bσuk+1 + zn−1
k+1 + Cνk+1
i
, k = 1, . . . , K −1
Box constraints:
xk ∈[xmin, xmax], k = 1, . . . , K
uk ∈[umin, umax], k = 1, . . . , K
Path constraints:
 
[yk, zk], xk tan γgs
!
∈K3, k = 1, . . . , K
Virtual control and trust region constraints:
∥νk∥2 ≤κk, k = 1, . . . , K
∥κ∥2 ≤sκ
(xk −xn−1
k
)⊤(xk −xn−1
k
) + (uk −un−1
k
)⊤(uk −un−1
k
) ≤ηk, k = 1, . . . , K
∥η∥2 ≤sη
∥σ −˜σ∥≤ησ
K−1
X
k=1
∆τ
2
h
ℓn convex,k(xk, uk) + ℓn convex,k+1(xk+1, uk+1)
i
≤sR
Boundary conditions:
r(0) = r0, v(0) = v0, m(0) = m0
r(1) = rf v(1) = vf
α(1) = 0
β(1) = 0
βy,i(1) = 0
βz,i(1) = 0
i = 1, . . . , 4
Chair of Space Mobility and Propulsion
|
Technical University Munich
55

6. Online Guidance
6.2.5 Nominal SCP Validation
The problem is initialized with the line approach described earlier, and the resulting trajectories are evaluated to assess the performance of the SCP algorithm. Position, velocity and mass (Figures 6.2a and 6.2b) together with propulsive and aerodynamic controls (Figures 6.3a and 6.4a) closely match the nominal results. All constraints remain satisfied, and the final integral value differs by only 0.48% as shown in Table 6.3, confirming the validity of the proposed approach.
Small discrepancies appear in the control profiles: most notably, a lower thrust rate during the first and only thrust bang, which results from the higher penalization weight for the thrust derivative in SCP
(Table 6.2), and small differences in the TVC and aerodynamic angles at the beginning of the trajectory, which remain well below 0.5◦. Both the thrust and TVC rate profiles (Figure 6.3b) appear noisier than in the nominal case, despite the thrust rate integral being penalized more heavily in SCP. These differences can be attributed to several factors, including the distinct linearization and dynamics approximations, the convergence conditions, and the discretization schemes; in particular, whereas SCP applies a uniform trapezoidal discretization, GPOPS-II employs an hp-adaptive pseudospectral method and directly solves the original nonconvex formulation using the NLP solver IPOPT [90].
A further distinction arises in the cost function formulation. While the NLP directly minimizes the integral cost in (5.8), the SCP objective also includes penalty terms associated with virtual controls and trust region slack. In addition, a small penalty on the final time is introduced, as numerical experimentation showed that this improves the agreement with the GPOPS-II solution. The treatment of the integral itself differs in two ways: the torque contributions are included in linearized form, and the weight on the thrust-rate term is increased from 10−3 to 10−1. Figure 6.5 shows the evolution of the slack variables for the virtual controls, trust region, and integral, together with the value of σ, across the SCP iterations.
Starting from the initialization at σ = 1, the time-dilation coefficient increases to its realistic value of about 6 within the first eight iterations. During this phase, the virtual-control slack variable remains non-negligible, since the discretized dynamics constraint in (6.39) cannot be satisfied without artificial inputs. Once σ stabilizes around 6, the magnitude of the virtual controls rapidly vanishes to essentially zero (numerical values on the order of 10−18, i.e. machine precision). From that point onward, both the integral slack and the trust region slack decrease, leading to convergence at iteration 13.
Table 6.2: SCP parameters
Integral weights
Slack weights
Other parameters wT
1
wη
10−1
ϵη
10−2
w ˙T
2 × 10−1
wν
105
ϵν
10−9
w ˙βy ˙βz
5 × 10−1
wσ
10−2
Nmax
30
w ˙α ˙β
6
wR
10
ησ
0.5
wprop
τ
10−2
waero
τ
10−4
Chair of Space Mobility and Propulsion
|
Technical University Munich
56

6. Online Guidance
Table 6.3: Integral values comparison
Term
GPOPS
SCP
Difference (%)
Total
3.35 × 10−2
3.33 × 10−2
−0.48
T
2.39 × 10−2
2.41 × 10−2
+0.63
˙T
9.76 × 10−4
5.95 × 10−4
−39.05
˙
TVC
2.64 × 10−3
2.49 × 10−3
−5.79
˙
aero
5.91 × 10−3
6.14 × 10−3
+3.79
Propulsive torque
2.7 × 10−5
2.4 × 10−5
−11.10
Aero torque
1.0 × 10−6
1.0 × 10−6
+2.66
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
(a) Position and velocity.
0
10
20
30
40
50
60
3700
3800
3900
4000
4100
4200
4300
(b) Mass.
Figure 6.2: Comparison of position, velocity and mass states.
Chair of Space Mobility and Propulsion
|
Technical University Munich
57

6. Online Guidance
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
(a) Thrust and TVC.
0
50
0
0.5
1
103
0
50
0
0.5
1
103
0
50
0
0.5
1
103
0
50
0
0.5
1
103
0
50
0
0.5
1
103
0
50
-1
-0.5
0
0.5
1
0
50
-0.1
-0.05
0
0
50
-0.1
-0.05
0
0
50
-0.1
-0.05
0
0
50
-0.1
-0.05
0
0
50
-1
-0.5
0
0.5
1
0
50
0
0.05
0.1
0.15
0
50
0
0.05
0.1
0.15
0
50
0
0.05
0.1
0.15
0
50
0
0.05
0.1
0.15
(b) Thrust and TVC rates.
Figure 6.3: Comparison of propulsive quantities: thrust magnitudes, TVC deflections, and rates.
Chair of Space Mobility and Propulsion
|
Technical University Munich
58

6. Online Guidance
0
10
20
30
40
50
60
-2.5
-2
-1.5
-1
-0.5
0
0
10
20
30
40
50
60
0
1
2
3
4
(a) Aerodynamic angles.
0
10
20
30
40
50
60
0
0.02
0.04
0.06
0
10
20
30
40
50
60
-0.08
-0.06
-0.04
-0.02
0
(b) Aerodynamics rates.
Figure 6.4: Comparison of aerodynamic quantities: aerodynamic angles and rates.
Chair of Space Mobility and Propulsion
|
Technical University Munich
59

6. Online Guidance
0
2
4
6
8
10
12
10-18
100
0
2
4
6
8
10
12
10-2
100
0
2
4
6
8
10
12
10-2
100
0
2
4
6
8
10
12
0
2
4
6
Figure 6.5: Evolution of the cost function components across SCP iterations.
6.2.6 Feasibility Check
The same feasibility check described in Section 5.1.9 confirms that the optimized trajectory satisfies the full nonlinear dynamics, with errors on the order of 3m in position as shown in Figure 6.6a and 6.6b.
6.3 Fault-Tolerant Sequential Convex Programming
In the following, the Fault-Tolerant Sequential Convex Programming (FT-SCP) scheme is introduced.
The objective is to extend the standard SCP formulation so that it can accommodate the degraded conditions resulting from a fault. The description begins with the definition of the fault cases, the information available at the time of the fault, followed by the modifications applied to the optimization problem in order to ensure feasibility and robustness under the new system constraints, ultimately allowing the computation of a trajectory consistent with the degraded system.
Chair of Space Mobility and Propulsion
|
Technical University Munich
60

6. Online Guidance
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
(a) Position and velocity propagation.
0
10
20
30
40
50
60
0
0.5
1
1.5
2
2.5
3
(b) Position error.
Figure 6.6: Validation of SCP solution: discrepancy between Runge-Kutta and SCP states.
Chair of Space Mobility and Propulsion
|
Technical University Munich
61

6. Online Guidance
6.3.1 Fault Scenarios
Based on the trends highlighted in [38], and similarly to [59], the failure cases considered in this work are listed in Table 6.4. Catastrophic failures are excluded, since both FTC and FTG would not be effective in such cases.
Propulsion failures are modeled as deviations from nominal thrust delivery by the propulsion system. Two distinct types are considered: total thrust failures (F1), where one engine shuts down and is unable to deliver any thrust, and partial thrust failures (F2), where one engine operates with degraded performance, being able to deliver only a certain percentage of its nominal maximum thrust. In addition, TVC actuator failures with jamming behavior are considered. These are represented as both TVC deflection angles being stuck at a fixed position (F3).
In [59], severe thrust loss scenarios are handled through an FTG approach, while all other failures, as well as their combinations, are managed by FTC strategies that integrate control allocation recovery, innerloop TVC control recovery, and FTC recovery. In the latter case, the nominal TVC controller is replaced by an FTC-based controller designed to be robust against propulsion failures.
In the present work, the emphasis is placed on the guidance aspect. Consequently, all failure modes are handled exclusively through the recovery guidance action.
For the purposes of this study, it is assumed that all types of failures are detected, isolated, and assessed instantly. In other words, FDI is considered ideal, with the fault type and its location identified without delay. Furthermore, guidance reconfiguration is assumed to occur immediately after FDI, and the recovery guidance is considered capable of delivering a solution with no time delay.
Table 6.4: Failure Cases
Name
Identifier
Total engine loss
F1
Thrust degradation
F2
TVC jamming
F3
To inform the guidance algorithm, the relevant information is passed at the time of the fault. This is achieved through the Fault Information Bus, which collects the parameters required for the problem reconfiguration and is summarized in Table 6.5.
Table 6.5: Information contained in the fault information bus.
Parameter
Units
Description
Fault time s
Time of fault occurrence
Thrust motor identifier
–
Index of faulty engine (1–5)
TVC motor identifier
–
Index of faulty actuator (1–4)
Thrust status
–
0 = Nominal, 1 = F2, 2 = F1
TVC status
–
0 = Nominal, 1 = F3
Thrust degradation
%
Value between
 Tmin,nom
Tmax,nom , 0.95

of nominal thrust
TVC stuck angles deg
Fixed deflections [βjam y
, βjam z
]
Chair of Space Mobility and Propulsion
|
Technical University Munich
62

6. Online Guidance
6.3.2 Final Position Relaxation
Similarly to the approach in [66], the standard SCP formulation is modified in the treatment of the horizontal components of the final position.
Instead of enforcing them as hard constraints, they are relaxed into soft constraints of the form
|y(tf)| ≤sy (6.58)
|z(tf)| ≤sz (6.59)
with the corresponding slack variables added to the cost function multiplied by a weight:
J = wpos(sy + sz) + wηsη + wκsκ + wσσ + wR sR
(6.60)
This modification maximizes feasibility at the expense of allowing a potential error in the final horizontal landing position. The relaxation is motivated by the fact that, if the initial state error is larger than expected and resources are degraded due to the fault, it may be preferable to tolerate a bounded horizontal error rather than risk non-convergence. For this reason, the glideslope constraint is modified accordingly such that the origin of the cone shifts with the final lateral position.

h y −y(tf)
z −z(tf)
i⊤
2
≤
x tan γgs (6.61)
6.3.3 Box Constraints & Final Boundary Condition Update
A complete thrust loss in one engine (F1) is handled by setting the minimum and maximum thrust of the affected engine to zero. In addition, the corresponding thrust rate, TVC deflection, and TVC rate constraints are also set to zero.
T i min = T i max = 0,
˙T i min = ˙T i max = 0
(6.62)
βmin y,i = βmax y,i
= 0,
βmin z,i = βmax z,i
= 0
(6.63)
˙βmin y,i = ˙βmax y,i
= 0,
˙βmin z,i = ˙βmax z,i
= 0
(6.64)
A thrust degradation failure (F2) is addressed by scaling the box constraint on the maximum thrust of the affected engine according to the degradation factor ξi. For instance, if engine 3 experiences a partial thrust fault with 50% degradation, then its maximum thrust is limited to 50% of the nominal value.
T i max = ξi T i max,nom,
T i min,nom
T imax,nom
< ξi < 1
(6.65)
In the event of a TVC actuator jamming fault (F3),with jammed values βi jam,y and βi jam,z, the corresponding deflection angles are fixed at the values reported in the fault information bus, and the box constraints on the affected TVC angles and rates are updated accordingly to reflect the jammed condition.
βmin y,i = βmax y,i
= βjam y,i ,
βmin z,i = βmax z,i
= βjam z,i (6.66)
˙βmin y,i = ˙βmax y,i
= 0,
˙βmin z,i = ˙βmax z,i
= 0
(6.67)
Chair of Space Mobility and Propulsion
|
Technical University Munich
63

6. Online Guidance
As a consequence, the final boundary condition on the TVC angles specified in Table 5.2 is removed, allowing the healthy actuators to land with nonzero deflections to compensate for the jammed actuator even at touchdown, as illustrated in Figure 6.7.
6.3.4 Initialization
At tfault, the FT-SCP receives as input the current augmented state, which includes the vehicle position, velocity, and mass, together with the control inputs applied by the controller. These quantities are scaled according to Table 5.3 and are then used as initial boundary conditions (IBCs) for the SCP problem. To better reflect the actual system behavior, the formulation enforces not only the state of the rocket at the time of the fault but also constrains the augmented state variables to coincide with the commands being applied by the controller. This prevents the guidance solution from returning control profiles that differ significantly from the current commanded values.
In order to accelerate convergence, the standard line-initialization approach is replaced with a scheme that exploits the nominal trajectory. Specifically, the algorithm is initialized using slices of the nominal state and control trajectories from tfault to the final time, modified to reflect the specific fault condition.
The final time guess is also initialized with the corresponding nominal value.
6.3.5 Performance Under Fault Scenarios
The performance of FT-SCP is evaluated by comparing its solutions under the different fault scenarios with the nominal solution from Chapter 5. In this case, the objective is not to reproduce the nominal trajectory, but to analyze how the state and control profiles evolve under the imposed fault conditions.
F1 – Total Thrust Loss
At tfault = 15 s, engine 4 shuts down completely. To compensate for the lost engine, the remaining healthy engines ignite approximately 10 s earlier than in the nominal case, reach maximum thrust, and sustain it for a longer duration, as shown in Figure 6.9a, which results in an additional propellant consumption of about 200 kg (Figure 6.8b). The resulting position and velocity profiles (Figure 6.8a) demonstrate that, despite the degraded propulsion system, the vehicle achieves a soft landing at the pad center, with vertical touchdown velocity of −1 m/s and negligible lateral velocity components.
Chair of Space Mobility and Propulsion
|
Technical University Munich
64

6. Online Guidance
Initialization
F2
F1
F3
Fault Info
Bus x0
Scale
Constraints Update 
F2
F1
Final BC
F3
F3
Engine ID
Thrust Status
TVC Status
Fault Time
SCP
Descale
FT-SCP
Figure 6.7: FT-SCP algorithm: integration of fault information.
Chair of Space Mobility and Propulsion
|
Technical University Munich
65

6. Online Guidance
0
10
20
30
40
50
60
70
0
2000
4000
6000
0
10
20
30
40
50
60
70
-200
-150
-100
-50
0
0
10
20
30
40
50
60
70
0
100
200
300
0
10
20
30
40
50
60
70
-15
-10
-5
0
0
10
20
30
40
50
60
70
0
50
100
150
200
0
10
20
30
40
50
60
70
-10
-5
0
(a) Position and velocity.
0
10
20
30
40
50
60
70
3500
3600
3700
3800
3900
4000
4100
4200
4300
(b) Mass.
Figure 6.8: FT-SCP solution under F1 (total thrust loss): position, velocity, and mass evolution.
Chair of Space Mobility and Propulsion
|
Technical University Munich
66

6. Online Guidance
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
(a) Propulsive controls.
0
10
20
30
40
50
60
70
-2.5
-2
-1.5
-1
-0.5
0
0
10
20
30
40
50
60
70
0
1
2
3
4
(b) Aerodynamic controls.
Figure 6.9: FT-SCP solution under F1 (total thrust loss): controls evolution.
F2 – Partial Thrust Loss
A thrust degradation fault with degradation factor ξ1 = 60% occurs in engine 1 at tfault = 25 s. The healthy engines respond by transitioning to the maximum thrust immediately at the fault time, while the degraded engine approximately 10 s later, reach maximum thrust, and sustain it for a longer duraChair of Space Mobility and Propulsion
|
Technical University Munich
67

6. Online Guidance tion, as shown in Figure 6.11a, which results in an additional propellant consumption of about 30 kg (Figure 6.10b). The resulting position and velocity profiles are shown in Figure 6.10a.
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
(a) Position and velocity.
0
10
20
30
40
50
60
3600
3700
3800
3900
4000
4100
4200
4300
(b) Mass.
Figure 6.10: FT-SCP solution under F2 (thrust degradation): position, velocity, and mass evolution.
Chair of Space Mobility and Propulsion
|
Technical University Munich
68

6. Online Guidance
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
0
1
2
3
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
(a) Propulsive controls.
0
10
20
30
40
50
60
-2.5
-2
-1.5
-1
-0.5
0
0
10
20
30
40
50
60
0
1
2
3
4
(b) Aerodynamic controls.
Figure 6.11: FT-SCP solution under F2 (thrust degradation): controls evolution.
F3 – TVC Jamming
This case considers a fault occurring at tfault = 5 s, where the TVC angles of engine 5 become stuck at 2.9◦and −3.8◦in the y and z axes, respectively. As a result, the thrust behavior of the remaining healthy engines remains essentially unchanged, leading to a final mass very close to the nominal case
Chair of Space Mobility and Propulsion
|
Technical University Munich
69

6. Online Guidance (Figure 6.12b). The TVC angles of the healthy engines adapt accordingly: the βy angles take slightly smaller values to counteract the positive βy,5, while the βz angles increase slightly to offset the negative
βz,5. In contrast, the thrust profile of the faulty engine deviates significantly from the nominal one. Its thrust never reaches or maintains the maximum value and gradually decreases, returning to the minimum at touchdown (Figure 6.13a).
0
10
20
30
40
50
60
0
2000
4000
6000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
0
100
200
300
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
0
50
100
150
200
0
10
20
30
40
50
60
-10
-5
0
(a) Position and velocity.
0
10
20
30
40
50
60
3700
3800
3900
4000
4100
4200
4300
(b) Mass.
Figure 6.12: FT-SCP solution under F3 (TVC jamming): position, velocity, and mass evolution.
Chair of Space Mobility and Propulsion
|
Technical University Munich
70

6. Online Guidance
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-1
0
1
2
3
0
20
40
60
-1
0
1
2
3
0
20
40
60
-1
0
1
2
3
0
20
40
60
0
1
2
3
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
0
20
40
60
-4
-3
-2
-1
0
(a) Propulsive controls.
0
10
20
30
40
50
60
-2.5
-2
-1.5
-1
-0.5
0
0
10
20
30
40
50
60
0
1
2
3
4
(b) Aerodynamic controls.
Figure 6.13: FT-SCP solution under F3 (TVC jamming): controls evolution.
Chair of Space Mobility and Propulsion
|
Technical University Munich
71

7. Verification & Validation
7 Verification & Validation
The performance of the FT-SCP algorithm is evaluated in a closed-loop simulation environment. The plant model corresponds to the one described in Chapter 4, while the nominal controller and reference trajectory are those presented in Chapter 5. Uncertainty in aerodynamic parameters, atmospheric conditions, and initial states is introduced according to the dispersions listed in Table 5.4.
7.1 Implementation Details
The closed-loop simulations are implemented in Simulink and the FT-SCP algorithm is realized as a
MATLAB [89] system block inside a triggered subsystem, with the trigger defined by the fault time. When the simulation time reaches tfault, the subsystem is activated and the FT-SCP is executed. Problem 1
is assembled with CVX, a package for specifying and solving convex programs [24, 35] and the resulting
SOCP is solved using the convex optimization solver MOSEK [8].
The plant dynamics evolve in continuous time, while guidance and control operate in discrete time at a frequency of 50 Hz. Each simulation trial begins with sampled aerodynamic uncertainties, initial dispersions, and fault. At t = tfault, the plant model is updated to reflect the appropriate faulty dynamics.
The FT-SCP is then triggered through the corresponding subsystem and computes a new reference trajectory, once it returns a feasible solution, the nominal reference trajectory used by the LQR controllers is switched to the FT-SCP generated one. Both the state and control trajectories are stored in lookup tables scheduled with respect to time.
The scheduling parameter is reset to zero at t = tfault and synchronized with the time vector produced by FT-SCP.
7.2 Study 1: Single Faults
7.2.1 Goal
The objective of this study is to evaluate the impact of individual faults on mission success. Each fault type (F1, F2, F3) is analyzed separately in order to:
• Assess the robustness of the FT-SCP algorithm against initial dispersions.
• Gain insight into how specific fault modes influence the mission outcome.
• Investigate the effect of a realistic guidance computation delay on landing performance.
Chair of Space Mobility and Propulsion
|
Technical University Munich
73

7. Verification & Validation
7.2.2 Setup
In each campaign, the fault time tfault is varied as a percentage of the reference trajectory duration. For every fault type, three Monte Carlo campaigns are performed under the same aerodynamic uncertainties and initial condition dispersions:
Nominal: The FT-SCP is not executed; the closed-loop simulation runs with the constant nominal reference trajectory and controller.
Recovery: The FT-SCP is triggered at tfault; once the guidance has converged, the reference trajectory is immediately switched to the recovery solution.
Recovery Delay: The FT-SCP is triggered at tfault; once the guidance has converged, the reference trajectory is switched to the recovery solution only after a simulated delay of tdelay.
A mission is considered successful if it results in a soft pinpoint landing, defined by the requirements listed in Table 5.5.
The randomized parameters for each fault type are summarized in 7.1.
Table 7.1: Randomized parameters for single-fault cases F1–F3
Parameter
F1
F2
F3
Fault time
U(0, 80% tref)
U(0, 80% tref)
U(0, 80% tref)
Engine number
U(1, 5)
U(1, 5)
U(2, 5)
Fault-specific parameter
–
U
 Tmin,nom
Tmax,nom , 0.95


U(−5◦, 5◦), U(−5◦, 5◦)

F1: One Engine Loss
Figure 7.1 summarizes the results for fault type F1. As expected, no nominal cases achieve a successful landing, since tracking the original reference trajectory becomes impossible with only four engines. The nominal LQR controller therefore fails, as it continues attempting to follow a trajectory that cannot be realized after an engine loss. The FT-SCP guidance converges in approximately 78% of the runs, i.e.
a feasible solution is found. However, only 65% of the closed-loop simulations satisfies the soft landing requirements. This discrepancy can be attributed to the limited authority of the nominal LQR controller, which is unaware of the engine failure. As a result, the controller continues issuing commands in terms of thrust increments and TVC deflections to the inoperative engine, and this, in combination with large aerodynamic uncertainties, leads to a fraction of feasible guidance solutions that cannot be successfully tracked, resulting in missed landings. The effect of guidance delay is relatively minor, reducing the overall success rate by only about 1.5%. Figures 7.2a and 7.2b show the 3D position trajectories of converged cases. The missed landings in the recovery cases can already be seen here, and even more clearly in
Figure 7.3a, which shows the lateral positions at landing color-coded by touchdown vertical velocity.
This highlights once again the difference with the guidance solutions in Figure 7.3b, where the relaxation of the final lateral position constraints introduced in (6.58) and (6.59) is in practice not exploited, as all converged FT-SCP solutions remain very close to the landing pad center. It can be observed from
Figure 7.4, which shows the final mass at landing for all converged cases, that the average final mass is
Chair of Space Mobility and Propulsion
|
Technical University Munich
74

7. Verification & Validation
Figure 7.1: F1 (total thrust loss): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
0
2000
400
600
4000
400
200
6000
200
0
0
-200
(a) Closed-loop recovery trajectories.
0
600
2000
600
400
4000
400
200
200
6000
0
0
-200
(b) FT-SCP reference solutions.
Figure 7.2: F1 (total thrust loss): 3D landing trajectories of all converged cases.
Chair of Space Mobility and Propulsion
|
Technical University Munich
75

7. Verification & Validation
0
50
100
150
200
250
-40
-20
0
20
40
60
80
100
120
140
160
-16
-14
-12
-10
-8
-6
-4
-2
Landing Pad
Final Positions
Target (a) Closed-loop recovery cases.
-15
-10
-5
0
5
10
15
-10
-5
0
5
10
-2
-1.8
-1.6
-1.4
-1.2
-1
-0.8
-0.6
-0.4
-0.2
Landing Pad
Final Positions
Target (b) FT-SCP solutions.
Figure 7.3: F1 (total thrust loss): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity.
Chair of Space Mobility and Propulsion
|
Technical University Munich
76

7. Verification & Validation about 200 kg lower than in the nominal case. Nevertheless, sufficient propellant remains at touchdown, since the dry mass of the vehicle is 2750 kg.
0
10
20
30
40
50
60
70
80
3200
3300
3400
3500
3600
3700
3800
3900
Figure 7.4: F1 (total thrust loss): Final mass at landing versus fault time for all converged cases.
Figure 7.5a highlights a clear dependency between guidance convergence and fault time: beyond 60%
of the nominal flight duration, the algorithm fails to converge in most cases. This behavior does not indicate a weakness of the algorithm but reflects the physical infeasibility of the problem itself.
As shown in Figure 6.9a, the total loss of one engine is compensated by firing the remaining healthy engines in advance (when compared to the nominal case) and for a longer duration. By the time 60% of the trajectory has elapsed, however, the nominal profile has already commanded the engines to ramp from minimum to maximum thrust (Figure 5.2a). Given the vehicle’s altitude and vertical velocity at this stage, even firing all four remaining engines at full thrust cannot generate sufficient deceleration to ensure a safe landing. The lack of convergence therefore stems from the absence of a feasible solution rather than numerical difficulties. This interpretation is further supported by the behavior of the virtual control slack variables shown in Figure 7.5b. For the non-converged cases, the slacks remain large at the final iteration, preventing satisfaction of the convergence criterion (6.56). In other words, the discretized dynamics in (6.39) cannot be enforced without substantial artificial inputs, confirming that the underlying problem is infeasible.
It is important to note that the orange (non-converged) dots with values well below 0, similar to those of the blue converged cases, can be misleading. This does not mean that the algorithm failed to converge despite a feasible solution.
One must recall that in order to converge, the algorithm also includes the condition on the trust-region
Chair of Space Mobility and Propulsion
|
Technical University Munich
77

7. Verification & Validation difference between subsequent iterations (6.57). If this difference is not sufficiently small, the algorithm is considered to have not converged. This is precisely the case here: although a trajectory with negligible virtual controls is found, it is obtained through a large deviation in states and controls from the previous iteration rather than through a small step within the linearized region. For this reason, the solution is still considered infeasible and therefore the algorithm does not reach convergence.
(a) Convergence versus fault time.
0
20
40
60
80
10-20
10-15
10-10
10-5
100
(b) Virtual controls slacks.
Figure 7.5: F1 (total thrust loss): FT-SCP convergence analysis.
All cases with a fault occurring at tfault ≳60% tref for which the guidance converges can be explained by looking at Figure 7.6, which shows the state error at the fault time, i.e. the deviation between the simulation state (the FT-SCP initial condition) and the nominal reference trajectory. Figure 7.6a illustrates this error in terms of vertical position, while Figure 7.6b shows the corresponding vertical velocity error.
From Figure 7.6a, it is clear that all converged cases share a common feature: the vertical position error is significant, on the order of several hundred meters. At the same time, Figure 7.6b indicates that these cases exhibit higher downward velocity, though not excessively large. In practice, this means that the vehicle is descending faster but still retains additional altitude margin compared to the nominal trajectory.
This combination provides more space and time for the remaining healthy engines to be fired in order to decelerate, which explains why the algorithm converges in these scenarios.
0
20
40
60
80
-500
0
500
1000
1500
(a) Vertical position error.
0
20
40
60
80
-50
0
50
(b) Vertical velocity error.
Figure 7.6: F1 (total thrust loss): State errors at fault time for converged and non-converged cases.
Chair of Space Mobility and Propulsion
|
Technical University Munich
78

7. Verification & Validation
F2: Degraded thrust
Figure 7.7 summarizes the results for fault type F2. In this case some nominal cases are still able to land, mainly those with a mild thrust degradation factor that can be compensated by the controller. Figure 7.8a
Figure 7.7: F2 (thrust degradation): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
highlights how the dispersions at landing are much smaller than for F1 and track the guidance results (Figure 7.8b) much better. The reason is that in this failure mode the faulty engine is never completely inoperative. Even at maximum degradation (i.e., when the faulty engine can only deliver its minimum thrust), the corresponding TVC still provides some control authority, though with reduced effectiveness.
This contrasts with F1, where the fault disables the engine entirely and removes the contribution of its
TVCs.
-30
-20
-10
0
10
20
30
-10
0
10
20
30
-25
-20
-15
-10
-5
Landing Pad
Final Positions
Target (a) Closed-loop recovery cases.
-15
-10
-5
0
5
10
15
-10
-5
0
5
10
-2
-1.5
-1
-0.5
Landing Pad
Final Positions
Target (b) FT-SCP solutions.
Figure 7.8: F2 (thrust degradation): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity.
Chair of Space Mobility and Propulsion
|
Technical University Munich
79

7. Verification & Validation
F3: TVC jamming
Figure 7.9 summarizes the results for fault type F3. As in the case of F2, some nominal runs are still able to achieve a successful landing. These are mainly cases with low aerodynamic uncertainties, small initial dispersions, and mild TVC faults, conditions under which the feedback controller is still able to compensate effectively.
Figure 7.9: F3 (TVC jamming): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
Figure 7.10b shows the FT-SCP solution’s lateral positions at landing, in which it can be noticed that the final position, even if with limited deviation, is actually exploited for this case, with some solutions landing around 1-2 meters from the origin. Figure 7.11a shows that the sensitivity of convergence to the fault time is reduced compared to F1, with many runs still converging even towards the end of the interval, close to 80% of the flight time. At the same time, a higher number of early-fault cases (in terms of tfault) fail to converge. Most of these can be attributed to situations where large TVC stuck angles are combined with significant lateral dispersions, preventing the algorithm from finding a feasible solution capable of steering the vehicle back to the landing pad.
As an example, a representative early non-converged case is summarized in Table 7.2. The combination of the initial conditions and the angles at which the TVC is stuck makes the problem of returning, or even approaching the landing pad, infeasible. Such a set of initial conditions can nonetheless produce a solution if the weight on the final lateral position relaxation is reduced. For instance, decreasing this weight from its original value of 50 to 1 leads to the results shown in Figure 7.12, where the algorithm is able to converge to a feasible solution with lateral touchdown positions of 100 m and 150 m in the y and z directions, respectively. The corresponding propulsive and aerodynamic control inputs are shown in Figure 7.13.
Chair of Space Mobility and Propulsion
|
Technical University Munich
80

7. Verification & Validation
-50
0
50
100
-20
0
20
40
60
80
100
120
-25
-20
-15
-10
-5
Landing Pad
Final Positions
Target (a) Closed-loop recovery cases.
-15
-10
-5
0
5
10
15
-10
-5
0
5
10
-2
-1.5
-1
-0.5
Landing Pad
Final Positions
Target (b) FT-SCP solutions.
Figure 7.10: F3 (TVC jamming): Lateral touchdown positions at the landing pad color-coded by vertical touchdown velocity.
Chair of Space Mobility and Propulsion
|
Technical University Munich
81

7. Verification & Validation
Table 7.2: Representative failed case (F3): Initial conditions and fault parameters
Parameter
Value
Unit r0
[43459, 489.2, 485.6]T
[m]
v0
[−165.5, 1.2, 10.2]T
[m/s]
m0
4233
[kg]
TVC motor ID
4
[–]
TVC stuck angle
[2.9, −3.8]
[deg]
Fault time
3.49
[s]
(a) Convergence versus fault time.
0
20
40
60
80
10-20
10-15
10-10
10-5
100
(b) Virtual controls slacks.
Figure 7.11: F3 (TVC jamming): FT-SCP convergence analysis.
0
10
20
30
40
50
60
0
1000
2000
3000
4000
0
10
20
30
40
50
60
-200
-150
-100
-50
0
0
10
20
30
40
50
60
100
200
300
400
0
10
20
30
40
50
60
-15
-10
-5
0
0
10
20
30
40
50
60
100
200
300
400
500
0
10
20
30
40
50
60
-10
-5
0
5
10
Figure 7.12: F3 (TVC jamming) relaxed solution: position and velocity.
Chair of Space Mobility and Propulsion
|
Technical University Munich
82

7. Verification & Validation
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
0
5
10
103
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
0
2
4
6
0
20
40
60
0
2
4
6
0
20
40
60
0
2
4
6
0
20
40
60
2
2.5
3
3.5
4
0
20
40
60
-1
-0.5
0
0.5
1
0
20
40
60
-6
-4
-2
0
0
20
40
60
-6
-4
-2
0
0
20
40
60
-6
-4
-2
0
0
20
40
60
-5
-4.5
-4
-3.5
-3
(a) Propulsive controls.
0
10
20
30
40
50
60
-10
-8
-6
-4
-2
0
0
10
20
30
40
50
60
0
2
4
6
8
10
(b) Aerodynamic controls.
Figure 7.13: F3 (TVC jamming) relaxed solution: controls.
Chair of Space Mobility and Propulsion
|
Technical University Munich
83

7. Verification & Validation
7.3 Study 2: Multiple Faults
The goal of this study is to characterize the impact of simultaneous thrust loss from multiple engines on the mission outcome. To this end, a dedicated Monte Carlo campaign is executed in which, at the fault time, three different thrust degradation factors are applied to three distinct engines, as summarized in
Table 7.3. Apart from the fault model, the simulation setup remains identical to the single-fault cases.
Table 7.3: Randomized parameters for multiple-fault case F4
Parameter
Distribution
Fault time
U(0, 80% tref)
Engine numbers distinct samples from U(1, 5)
Thrust degradation factors ξ1, ξ2, ξ3
U

Tmin,nom
Tmax,nom , 0.95

Figure 7.14 shows that the FT-SCP converged in about 55% of the runs, with the recovery and recoverydelay cases displaying similar results.
Figure 7.14: F2 (thrust degradation on multiple engines): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
Figure 7.15 highlights the key difference compared to the single-fault cases: the convergence trend with respect to fault time is much weaker. This can be attributed to the fact that severe thrust degradations may be unrecoverable even from the very beginning of the mission. To better understand how these faults affect the outcome, Figure 7.16 shows, in blue bars, the minimum thrust (as a percentage of the maximum nominal one) with which the rocket is able to land, plotted as a function of fault time, together with the number of converged runs for each interval. It is clear that as the fault occurs later in the descent (i.e., closer to the landing pad), the minimum thrust required for a successful landing increases, in other words, the engines cannot be too degraded. At the very beginning of the mission, even severe thrust
Chair of Space Mobility and Propulsion
|
Technical University Munich
84

7. Verification & Validation
Figure 7.15: F4 (thrust degradation on multiple engines): Soft landing success rates for nominal, recovery, recovery-delay, and converged FT-SCP cases.
degradations remain recoverable. In fact, some cases in which three faulty engines can only fire at their nominal minimum thrust, thereby reducing the overall available thrust to about 60%, still result in a safe landing.
Chair of Space Mobility and Propulsion
|
Technical University Munich
85

7. Verification & Validation
0
20
40
60
80
100
0
10
20
30
40
50
60
70
80
90
0
10
20
30
40
50
60
70
80
90
100
Figure 7.16: F4 (thrust degradation on multiple engines): Critical thrust as a function of fault time.
Chair of Space Mobility and Propulsion
|
Technical University Munich
86

8. Conclusions & Future Directions
8 Conclusions & Future Directions
This thesis set out to investigate how fault-tolerant guidance can be achieved for reusable launch vehicles during the powered descent and landing phase. To this end, two research questions were posed:
RQ1: How can an online guidance provide fault tolerance in presence of faults affecting the engines in reusable launch vehicles during the landing burn?
RQ2: To what extent (i.e., until when across the descent) is the contingency guidance effective?
The following sections provide direct answers to these questions, summarize the main contributions of the thesis, and outline directions for future research as well as the limitations of the approach.
In the first half of Chapter 6, the foundation of the convex framework for trajectory optimization was established, leading to the standard SCP algorithm that provided the baseline for a general online guidance method. This formulation was then expanded in the second half of the same chapter in order to address the first research question. In response to the first research question, this thesis has shown that sequential convex programming, extended to a fault-tolerant SCP (FT-SCP) formulation, enables the recomputation of feasible landing trajectories even in the presence of faults. By appropriately updating the
OCP formulation based on the fault information, the algorithm is able to compute a trajectory adapted to the reduced resources caused by the fault.
The second research question concerned the extent to which such contingency guidance remains effective across the descent.
Extensive Monte Carlo campaigns revealed that the FT-SCP approach achieves safe landings for a wide range of fault scenarios, provided that faults occur with sufficient altitude and time margin. For faults that resulted in a complete engine loss, it was shown that without guidance recomputation none of the runs concluded with a successful landing. After the introduction of recovery guidance, however, many of the previously failed runs became successful, although recovery guidance alone was not always sufficient to guarantee a soft landing.
In these cases, the baseline controller, designed under nominal conditions, was unable to track some of the feasible FT-SCP trajectories when aerodynamic uncertainties were significant. For other types of faults, such as partial thrust degradation and TVC jamming, the results demonstrated that recovery guidance was highly effective even when using the baseline controller.
The effectiveness of the approach decreases when faults occur later in the descent, as the combination of altitude, vertical velocity, and degraded conditions can make it physically impossible for the vehicle to decelerate sufficiently before touchdown. Overall, the method substantially extends the safe operating envelope of the vehicle, even though it cannot guarantee success in every possible scenario.
Chair of Space Mobility and Propulsion
|
Technical University Munich
87

8. Conclusions & Future Directions
8.1 Limitations
Despite the promising results, several limitations of the present work must be acknowledged. First, the
FT-SCP algorithm was only tested on a simulation model in which the plant dynamics closely matched those used in the OCP formulation.
A more thorough verification against full 6-DoF simulations is required to ensure that the generated trajectories are physically feasible when all degrees of freedom are considered. Furthermore, results in Chapter 7 showed that guidance computation delay had a minor impact on the outcome of the mission. However, it remains unknown whether this conclusion would be altered with the inclusion of attitude dynamics.
Second, the algorithm is highly sensitive to the numerical values chosen for the cost function weights.
Small variations in these values can drastically alter the solutions, and in some cases represent the difference between convergence and divergence.
This sensitivity complicates the tuning process and raises questions about robustness.
Third, although convex optimization was selected for its predictable runtime and convergence guarantees, the current implementation is not yet online-capable. Convergence times remain too long for real-time applications, primarily due to (i) numerical computation of Jacobians via finite differences, and (ii)
reliance on CVX as an intermediate modeling layer. Both aspects introduce significant computational overhead.
Finally, the entire work assumes instantaneous and error-free FDI. This assumption is clearly unrealistic and may lead to an overestimation of the practical effectiveness of the proposed approach.
8.2 Future Work
Several directions emerge naturally for future work.
First, extending the algorithm to a full 6-DoF
formulation of the dynamics would allow more realistic verification and potentially reveal new limitations of the current approach.
Second, reducing computational effort is critical for enabling online use. Promising directions include:
• Replacing aerodynamic lookup tables with polynomial approximations to allow for analytical expressions of the Jacobians, or applying automatic differentiation instead of computing Jacobians through finite differences.
• Implementing the algorithm in C and directly interfacing with the SOCP solver, removing CVX as a middle layer.
Third, combining trajectory reconfiguration with FTC strategies remains an open question. A systematic analysis is required to determine the range of faults for which a full guidance recomputation is beneficial, and the conditions under which a simpler FTC strategy would be sufficient, as well as how much the overall performance would improve from combining the two approaches.
Simulations with an actual
FDI module could provide further insight into how imperfect and delayed FDI would affect the mission outcome.
Finally, the mission scope considered in this thesis was restricted to the final PDL phase. Expanding the envelope to earlier mission stages, such as from the first descent burn, would provide more freedom and a
Chair of Space Mobility and Propulsion
|
Technical University Munich
88

8. Conclusions & Future Directions larger solution space for recovery. Likewise, allowing for adaptive mission objectives, such as diverting to an alternate landing pad in case the nominal site is no longer reachable, would significantly enhance the practical utility of the method. Along similar lines, the concepts explored here may also find applicability in the ascent phase, for example in connection with rescue-orbit strategies that have been proposed in the literature.
Chair of Space Mobility and Propulsion
|
Technical University Munich
89

Bibliography
Bibliography
Bibliography
[1]
(ESA), E. S. A.: Themis, https://www.esa.int/Enabling_Support/Space_Transportation/
Themis.
[2]
(ISRO), I. S. R. O.: Reusable Launch Vehicle – Technology Demonstrator (RLV-TD), %5Curl%
7Bhttps://www.isro.gov.in/RLVTD.html%7D.
[3]
A¸cıkme¸se, B.; Blackmore, L.: Lossless convexification of a class of optimal control problems with non-convex control constraints. Automatica, Vol. 47, No. 2, pp. 341–347, 2011.
[4]
A¸cıkme¸se, B.; Carson, J. M.; Blackmore, L.: Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem. IEEE Transactions on
Control Systems Technology, Vol. 21, No. 6, pp. 2104–2113, 2013.
[5]
A¸cıkme¸se, B.; Casoliva, J.; Carson, J.; Blackmore, L.: G-FOLD: A Real-Time Implementable Fuel
Optimal Large Divert Guidance Algorithm for Planetary Pinpoint Landing. LPI Contributions, pp. 4193–, June 2012.
[6]
Anderson, B. D. O.; Moore, J. B.: Optimal control: linear quadratic methods, USA: Prentice-Hall,
Inc., ISBN 0136385605, 1990.
[7]
Andersson, J. A. E.; Gillis, J.; Horn, G.; Rawlings, J. B.; Diehl, M.: CasADi – A software framework for nonlinear optimization and optimal control. Mathematical Programming Computation, Vol. 11,
No. 1, pp. 1–36, 2019.
[8]
ApS, M.: MOSEK Optimization Toolbox for MATLAB Manual. Version 11.0, 2025.
[9]
Baiocco, P.: Overview of reusable space systems with a look to technology aspects. Acta Astronautica, Vol. 189, pp. 10–25, Dec. 2021.
[10]
Bauer, W.; Rickmers, P.; Kallenbach, A.; Stappert, S.; Wartemann, V.; Hans-Joachim Merrem,
C.; Schwarz, R.; Sagliano, M.; Grundmann, J. T.; Flock, A.; Thiele, T.; Kiehn, D.; Bierig, A.;
Windelberg, J.; Ksenik, E.; Bruns, T.; Ruhe, T.; Els¨aßer, H.: DLR Reusability Flight Experiment
ReFEx. Acta Astronautica, Vol. 168, pp. 57–68, 2020.
[11]
Bazaraa, M. S.: Nonlinear Programming: Theory and Algorithms, Wiley Publishing, ISBN 1118857569,
2013.
[12]
Bedrossian, N.; Bhatt, S.; Lammers, M.; Nguyen, L.: Zero Propellant Maneuver: Flight Results for
180° ISS Rotation, NASA CP 2007-214158, Sept. 2007.
[13]
Benson, D. A.; Huntington, G. T.; Thorvaldsen, T. P.; Rao, A. V.: Direct Trajectory Optimization and Costate Estimation via an Orthogonal Collocation Method. Journal of Guidance, Control, and
Dynamics, Vol. 29, No. 6, pp. 1435–1440, 2006.
[14]
Betts, J.: Practical methods for optimal control and estimation using nonlinear programming. 2nd ed, Vol. 19, Jan. 2010.
Chair of Space Mobility and Propulsion
|
Technical University Munich
91

Bibliography
Bibliography
[15]
Betts, J. T.: Survey of Numerical Methods for Trajectory Optimization. Journal of Guidance,
Control, and Dynamics, Vol. 21, No. 2, pp. 193–207, 1998.
[16]
Bittner, L.: L. S. Pontryagin, V. G. Boltyanskii, R. V. Gamkrelidze, E. F. Mishechenko, The Mathematical Theory of Optimal Processes. VIII + 360 S. New York/London 1962. John Wiley & Sons.
Preis 90/–. ZAMM - Journal of Applied Mathematics and Mechanics / Zeitschrift f¨ur Angewandte
Mathematik und Mechanik, Vol. 43, No. 10-11, pp. 514–515, 1963.
[17]
Blue Origin: Blue Origin Makes Historic Rocket Landing, https://www.blueorigin.com/news/
blue-origin-makes-historic-rocket-landing, 2015.
[18]
Blue Origin: New Glenn Mission NG-1, https://www.blueorigin.com/missions/ng-1, 2025.
[19]
Bonalli, R.; Cauligi, A.; Bylard, A.; Pavone, M.: GuSTO: Guaranteed Sequential Trajectory optimization via Sequential Convex Programming. In: 2019 International Conference on Robotics and
Automation (ICRA), 2019.
[20]
Boyd, S.; Vandenberghe, L.: Convex Optimization, Cambridge University Press, 2004.
[21]
Bryson, A.: Applied Optimal Control: Optimization, Estimation and Control, CRC Press, ISBN
9781351465915, 2018.
[22]
Clark, S.: SpaceX Successfully Fires Satellites into Orbit but Loses Booster on Landing, https:
//spaceflightnow.com/2016/06/15/spacex-successfully-fires-satellites-into-orbitbut-loses-booster-on-landing/, 2016.
[23]
Cofield, C.: SpaceX Launches Falcon Heavy, World’s Most Powerful Rocket, on Historic Test Flight, https://www.space.com/39607-spacex-falcon-heavy-first-test-flight-launch.html,
2018.
[24]
CVX Research, I.: CVX: Matlab Software for Disciplined Convex Programming, version 2.0, Aug.
2012.
[25]
Darby, C. L.; Hager, W. W.; Rao, A. V.: An hp-adaptive pseudospectral method for solving optimal control problems. Optimal Control Applications and Methods, Vol. 32, No. 4, pp. 476–502, 2011.
[26]
Dietlein, I.; Bussler, L.; Stappert, S.; Wilken, J.; Sippel, M.: Overview of System Study on Recovery
Methods for Reusable First Stages of Future European Launchers. CEAS Space Journal, Vol. 17,
No. 1, pp. 71–88, 2025.
[27]
Dormand, J.; Prince, P.: A family of embedded Runge-Kutta formulae. Journal of Computational and Applied Mathematics, Vol. 6, No. 1, pp. 19–26, 1980.
[28]
Duchi, J.; Boyd, S.; Mattingley, J.: Sequential Convex Programming, https://web.stanford.
edu/class/ee364b/lectures/seq_notes.pdf, 2018.
[29]
Dumont, E.; Ecker, T.; Chavagnac, C.; Witte, L.; Windelberg, J.; Klevanski, J.; Giagkozoglou, S.:
CALLISTO - Reusable VTVL launcher first stage demonstrator. In: May 2018.
[30]
European Space Agency: History: Hermes Spaceplane (1987), https://www.esa.int/About_Us/
50_years_of_ESA/History_Hermes_spaceplane_1987, 1987.
[31]
Evaluation, S. S.; Team, A.: Surveyor I: Preliminary Results. Science, Vol. 152, No. 3730, pp. 1737–
1750, 1966.
[32]
Fari, S.; Seelbinder, D.; Theil, S.; Simpl´ıcio, P.: Robust Fault Detection and Isolation algorithms for TVC systems: An experimental test. In: 75th International Astronautical Congress, 2024.
Chair of Space Mobility and Propulsion
|
Technical University Munich
92

Bibliography
Bibliography
[33]
Fern´andez, L. A.; Wiedemann, C.; Braun, V.: Analysis of Space Launch Vehicle Failures and PostMission Disposal Statistics. Aerotec. Missili Spaz. Vol. 101, pp. 243–256, Sept. 2022.
[34]
Freeman, D. C.; Talay, T. A.; Austin, R. E.: Reusable Launch Vehicle Technology Program, NASATM-110473, 1996.
[35]
Grant, M.; Boyd, S.: Graph implementations for nonsmooth convex programs. In: Recent Advances in Learning and Control. Springer-Verlag Limited, pp. 95–110, 2008.
[36]
Greensite, A. L.: Analysis and Design of Space Vehicle Flight Control Systems. Volume VII: Attitude
Control During Launch, NASA-CR-826, 1967.
[37]
Hiriart-Urruty, J.; Lemarechal, C.: Convex Analysis and Minimization Algorithms I: Fundamentals.
Grundlehren der mathematischen Wissenschaften, Springer Berlin Heidelberg, ISBN 9783540568506,
1996.
[38]
Hussein, M.; De Weck, O.; Terakado, D.: Comprehensive Study of the International Space Launch
Industry: Programmatic Analysis and Technical Failures. In: IAF Space Transportation Solutions and Innovations Symposium, 2024.
[39]
Hwang, I.; Kim, S.; Kim, Y.; Seah, C. E.: A Survey of Fault Detection, Isolation, and Reconfiguration
Methods. IEEE Transactions on Control Systems Technology, Vol. 18, No. 3, pp. 636–653, 2010.
[40]
Isermann, R.; Ball´e, P.: Trends in the application of model-based fault detection and diagnosis of technical processes. Control Engineering Practice, Vol. 5, No. 5, pp. 709–719, 1997.
[41]
Jarre, F.: Interior-point methods for convex programming. Applied Mathematics and Optimization,
Vol. 26, No. 3, pp. 287–311, 1992.
[42]
Jones, A.: Long March 8 Debuts with Successful Launch of Nine Satellites, https : / / www .
nasaspaceflight.com/2020/12/long-march-8-debuts-nine-satellites/, 2020.
[43]
Josselyn, S.; Ross, I.: Rapid Verification Method for the Trajectory Optimization of Reentry Vehicles. Journal of Guidance Control and Dynamics - J GUID CONTROL DYNAM, Vol. 26, pp. 505–
508, May 2003.
[44]
Liu, X.: Fuel-Optimal Rocket Landing with Aerodynamic Controls. Journal of Guidance, Control, and Dynamics, Vol. 42, No. 1, pp. 65–77, 2019.
[45]
Liu, X.; Lu, P.; Pan, B.: Survey of convex optimization for aerospace applications. Astrodynamics,
Vol. 1, No. 1, pp. 23–40, 2017.
[46]
Liu, X.; Shen, Z.; Lu, P.: Entry Trajectory Optimization by Second-Order Cone Programming.
Journal of Guidance, Control, and Dynamics, Vol. 39, No. 2, pp. 227–241, 2016.
[47]
Lobo, M. S.; Vandenberghe, L.; Boyd, S.; Lebret, H.: Applications of second-order cone programming. Linear Algebra and its Applications, Vol. 284, No. 1, International Linear Algebra Society (ILAS) Symposium on Fast Algorithms for Control, Signals and Image Processing, pp. 193–228,
1998.
[48]
Lu, P.: Convex–Concave Decomposition of Nonlinear Equality Constraints in Optimal Control.
Journal of Guidance, Control, and Dynamics, Vol. 44, pp. 1–11, Oct. 2020.
[49]
Malyuta, D.; Reynolds, T.; Szmuk, M.; Lew, T.; Bonalli, R.; Pavone, M.; A¸cıkme¸se, B.: Convex
Optimization for Trajectory Generation, June 2021.
Chair of Space Mobility and Propulsion
|
Technical University Munich
93

Bibliography
Bibliography
[50]
Mao, Y.; Szmuk, M.; A¸cıkme¸se, B.: Successive Convexification: A Superlinearly Convergent Algorithm for Non-convex Optimal Control Problems, Apr. 2018.
[51]
Miao, X.; Song, Y.; Zhang, Z.; Gong, S.: Successive Convexification for Ascent Trajectory Replanning of a Multistage Launch Vehicle Experiencing Nonfatal Dynamic Faults. IEEE Transactions on
Aerospace and Electronic Systems, Vol. 58, No. 3, pp. 2039–2052, 2022.
[52]
Morio, V.: Design and development of an autonomous guidance law by flatness approach. Application to an atmospheric reentry mission. May 2009.
[53]
NASA: NASA Tipping Point Partnership with Blue Origin to Test Precision Lunar Landing Technologies.
[54]
National Aeronautics and Space Administration: Viking 1: Early Results, SP-406, 1976.
[55]
Nocedal, J.; Wright, S.: Numerical optimization. Springer series in operations research and financial engineering, New York, NY: Springer, ISBN 978-0-387-30303-1, 2006.
[56]
Orr, J.; Zwieten, T. V.: Robust, Practical Adaptive Control for Launch Vehicles. In: AIAA Guidance, Navigation, and Control Conference.
[57]
Pascucci, C. A.; Fernandez, A.; Jerez, J.; Bennani, S.; Jean-PhilippePreaud, ‡.; Bru, J.; Carlo;
Pascucci, A.: Design Simulation and Software Validation for On-board and Real-Time Optimal
Guidance. In: 2019.
[58]
Patterson, M. A.; Rao, A. V.: GPOPS-II: A MATLAB Software for Solving Multiple-Phase Optimal Control Problems Using hp-Adaptive Gaussian Quadrature Collocation Methods and Sparse
Nonlinear Programming. ACM Trans. Math. Softw. Vol. 41, No. 1, Oct. 2014.
[59]
Paulino, N.; Roche Arroyos, C.; Ferreira, L.; Pascucci, M.; Cachim, P.; Louren¸co, P.; Garc´ıa, J.;
Navarro-Tapia, D.; Marcos, A.; Mohamed, L.; Alexandre, P.; Simplicio, P.; Bennani, S.: Fault
Tolerant Control for a Cluster of Rocket Engines -Methods and outcomes for guidance and control recovery strategies in launchers. In: June 2023.
[60]
Rao, A.: A Survey of Numerical Methods for Optimal Control. Advances in the Astronautical
Sciences, Vol. 135, Jan. 2010.
[61]
http://www.falcon-m.com Rieck, M.; Bittner, M.; Gr¨uter, B.; Diepolder, J.; Piprek, P.; G¨ottlicher,
C.; Schwaiger, F.; Hosseini, B.; Schweighofer, F.; Akman, T.; Holzapfel, F.: FALCON.m User Guide.
version 1.32, 2024.
[62]
Ross, I. M.; Fahroo, F.: User’s Manual for DIDO 2001α: A MATLAB Application for Solving
Optimal Control Problems, AAS-01-03, 2001.
[63]
Runge–Kutta Methods. In: Numerical Methods for Ordinary Differential Equations. John Wiley
Sons, Ltd, chap. 3, pp. 137–316, ISBN 9780470753767, 2008.
[64]
Sagliano, M.: Generalized hp Pseudospectral-Convex Programming for Powered Descent and Landing. Journal of Guidance, Control, and Dynamics, Vol. 42, No. 7, pp. 1562–1570, 2019.
[65]
Sagliano, M.: Pseudospectral Convex Optimization for Powered Descent and Landing. Journal of
Guidance, Control, and Dynamics, Vol. 41, No. 2, pp. 320–334, 2018.
[66]
Sagliano, M.; Heidecker, A.; Far`ı, S.; Alfredo, M. H. J.; Schlotterer, M.; Woicke, S.; Seelbinder,
D.; Dumont, E.: Powered Atmospheric Landing Guidance for Reusable Rockets: the CALLISTO
studies. In: AIAA SCITECH 2024 Forum.
Chair of Space Mobility and Propulsion
|
Technical University Munich
94

Bibliography
Bibliography
[67]
Sagliano, M.; Heidecker, A.; Hern´andez, J. M.; Far`ı, S.; Schlotterer, M.; Woicke, S.; Seelbinder, D.;
Dumont, E.: Onboard Guidance for Reusable Rockets: Aerodynamic Descent and Powered Landing.
In: AIAA Scitech 2021 Forum.
[68]
Sagliano, M.; Hern´andez, J. A. M.; Farı, S.; Heidecker, A.; Schlotterer, M.; Woicke, S.; Seelbinder,
D.; Krummen, S.; Dumont, E.: Unified-Loop Structured H-Infinity Control for Aerodynamic Steering of Reusable Rockets. Journal of Guidance, Control, and Dynamics, Vol. 46, No. 5, pp. 815–837,
2023.
[69]
Sagliano, M.; Ishimoto, S.; Mac´es-Hern´andez, J. A.; Seelbinder, D.; Dumont, E.: Guidance and
Control Strategy for the CALLISTO Flight Experiment. In: July 2019.
[70]
Sagliano, M.; Lu, P.; Seelbinder, D.; Theil, S.: Analytical Treatise on Endo-Atmospheric FuelOptimal Rocket Landings. Journal of Guidance, Control, and Dynamics, Vol. 48, No. 3, pp. 450–
469, 2025.
[71]
Sagliano, M.; Seelbinder, D.; Theil, S.: SPARTAN: Rapid Trajectory Analysis via Pseudospectral
Methods. In: June 2021.
[72]
Sagliano, M.; Seelbinder, D.; Theil, S.; Im, S.; Lee, J.; Lee, K.: Booster Dispersion Area Management through Aerodynamic Guidance and Control. In: AIAA SCITECH 2022 Forum.
[73]
Sagliano, M.; Seelbinder, D.; Theil, S.; Lu, P.: Six-Degree-of-Freedom Rocket Landing Optimization via Augmented Convex–Concave Decomposition. Journal of Guidance, Control, and Dynamics,
Vol. 47, No. 1, pp. 20–35, 2024.
[74]
Scharf, D. P.; Regehr, M. W.; Vaughan, G. M.; Benito, J.; Ansari, H.; Aung, M.; Johnson, A.;
Casoliva, J.; Mohan, S.; Dueri, D.; A¸cikme¸se, B.; Masten, D.; Nietfeld, S.: ADAPT demonstrations of onboard large-divert Guidance with a VTVL rocket. In: 2014 IEEE Aerospace Conference, 2014.
[75]
Shampine, L. F.; Reichelt, M. W.: The MATLAB ODE Suite. SIAM Journal on Scientific Computing, Vol. 18, No. 1, pp. 1–22, 1997.
[76]
Siddiqi, A. A.: Beyond Earth: A Chronicle of Deep Space Exploration, 1958–2016, National Aeronautics and Space Administration, Office of Communications, NASA History Division, 2018.
[77]
Simplicio, P.; Marcos, A.; Bennani, S.: A Reusable Launcher Benchmark with Advanced Recovery
Guidance. In: 5th CEAS EuroGNC Conference, 2019.
[78]
Sissenwine; Teweles, S.; Dubin, M.: U. S. Standard Atmosphere, 1976. In: National Aeronautics and Space Administration, October 1976, pp. 1 - 227, NASA-TM-X-74335, NOAA-S/T 76-1562.
[79]
Slyuta, E.: CHAPTER 3 - The Luna program. In: Sample Return Missions. Elsevier, pp. 37–78,
ISBN 978-0-12-818330-4, 2021.
[80]
Smaili, H.; Breeman, J.; Lombaerts, T.; Stroosma, O.: A Benchmark for Fault Tolerant Flight
Control Evaluation. IFAC Proceedings Volumes, Vol. 42, No. 8, 7th IFAC Symposium on Fault
Detection, Supervision and Safety of Technical Processes, pp. 241–246, 2009.
[81]
Song, Z.-y.; Wang, C.; Theil, S.; Seelbinder, D.; Sagliano, M.; Liu, X.-f.; Shao, Z.-j.: Survey of autonomous guidance methods for powered planetary landing. Frontiers of Information Technology
& Electronic Engineering, Vol. 21, No. 5, pp. 652–674, 2020.
Chair of Space Mobility and Propulsion
|
Technical University Munich
95

Bibliography
Bibliography
[82]
Song, Z.; Pan, H.; Zhao, Y.; Yao, W.; He, Y.; Wang, C.: Reviews and Challenges in Reliability
Design of Long March Launcher Control Systems. AIAA Journal, Vol. 60, No. 2, pp. 537–550,
2022.
[83]
Song, Z.; Wang, C.; Gong, Q.: Joint dynamic optimization of the target orbit and flight trajectory of a launch vehicle based on state-triggered indices. Acta Astronautica, Vol. 174, pp. 82–93, 2020.
[84]
SpaceX: Starship Flight 5, https://www.spacex.com/launches/starship-flight-5, 2024.
[85]
Stevens, B.; Lewis, F.; Johnson, E.: Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems, Third Edition, ISBN 9781119174882, Oct. 2015.
[86]
Sutton, G.; Biblarz, O.: Rocket Propulsion Elements. A Wiley Interscience publication, Wiley, ISBN
9780471326427, 2001.
[87]
Szmuk, M.; Acikmese, B.: Successive Convexification for 6-DoF Mars Rocket Powered Landing with
Free-Final-Time. In: 2018 AIAA Guidance, Navigation, and Control Conference.
[88]
Szmuk, M.; Acikmese, B.; Berning, A. W.: Successive Convexification for Fuel-Optimal Powered
Landing with Aerodynamic Drag and Non-Convex Constraints. In: AIAA Guidance, Navigation, and Control Conference.
[89]
The MathWorks, Inc.: MATLAB, 2024.
[90]
W¨achter, A.; Biegler, L.; Lang, Y.-d.; Raghunathan, A.: IPOPT: An interior point algorithm for large-scale nonlinear optimization, Jan. 2002.
[91]
Wall, M.: SpaceX Successfully Lands Falcon 9 Rocket After Launching Satellites, https://www.
space.com/31420-spacex-rocket-landing-success.html, 2015.
[92]
Wang, Z.; Lu, Y.: Improved Sequential Convex Programming Algorithms for Entry Trajectory
Optimization. Journal of Spacecraft and Rockets, Vol. 57, No. 6, pp. 1373–1386, 2020.
[93]
Zolghadri, A.: The challenge of advanced model-based FDIR techniques for aerospace systems: 2011
situation. EUCASS Proceedings Series, Dec. 2013.
Chair of Space Mobility and Propulsion
|
Technical University Munich
96