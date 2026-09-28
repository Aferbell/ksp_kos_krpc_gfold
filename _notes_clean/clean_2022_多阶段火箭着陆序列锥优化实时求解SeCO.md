# arxiv_2212_00375_SeCO_multistage_rocket_landing

- source_file: `arxiv_2212_00375_SeCO_multistage_rocket_landing.pdf`
- pages: 8
- pdf_title_meta: 
- pdf_author_meta: 
- chars_extracted: 42043

## Detected headings (heuristic)

- 2.1 Time-interval dilation
- 2.2 Linearization
- 2.3 Discretization
- We leverage time-interval dilation to impose single-crossing
- 4.1 Virtual state
- 4.2 Conic subproblem
- 6.1 Common constraints
- Glideslope
- Tmax
- Tmin
- Solution
- Single-shot integration
- Interpolation
- 6.2 The unpowered coast phase, k ∈{1:kignition−1}
- 6.3 The high-thrust burn phase, k ∈{kignition :kswitch−1}
- 3 Tmin ≤Tk ≤3 Tmax (24)
- 6.4 The low-thrust burn phase, k ∈{kswitch :ktrigger−1}
- 6.5 The terminal descent phase, k ∈{ktrigger :N}
- 6.6 The discrete seco subproblem
- A real-time guidance solution obtained via pipg is shown
- ACKNOWLEDGEMENTS
- REFERENCES
- International

---

## Full text

Real-Time Sequential Conic Optimization for Multi-Phase Rocket Landing Guidance⋆
Abhinav G. Kamath∗Purnanand Elango∗Yue Yu∗∗
Skye Mceowen∗Govind M. Chari∗John M. Carson III∗∗∗
Beh¸cet A¸cıkme¸se∗
∗William E. Boeing Department of Aeronautics & Astronautics,
University of Washington, Seattle, WA 98195, USA
(e-mail: {agkamath, pelango, skye95, gchari, behcet}@uw.edu).
∗∗Oden Institute for Computational Engineering and Sciences, The
University of Texas at Austin, Austin, TX 78712, USA
(e-mail: yueyu@utexas.edu).
∗∗∗NASA Johnson Space Center, Houston, TX 77058, USA
(e-mail: john.m.carson@nasa.gov).
Abstract: We introduce a multi-phase rocket landing guidance framework that can handle nonlinear dynamics and does not mandate any additional mixed-integer or nonconvex constraints to handle discrete temporal events/switching. To achieve this, we first introduce sequential conic optimization (seco), a new paradigm for solving nonconvex optimal control problems that is entirely devoid of matrix factorizations and inversions. This framework combines sequential convex programming (SCP) and first-order conic optimization and can solve unified multi-phase trajectory optimization problems in real-time. The novel features of this framework are: (1) timeinterval dilation, which enables multi-phase trajectory optimization with free-transition-time; (2)
single-crossing compound state-triggered constraints, which are entirely convex if the trigger and constraint conditions are convex; (3) virtual state, which is a new approach to handling artificial infeasibility in SCP methods that preserves the shapes of the constraint sets; and, (4) the use of the proportional-integral projected gradient method (pipg), a high-performance first-order conic optimization solver, in tandem with the penalized trust region (ptr) SCP algorithm. We demonstrate the efficacy and real-time capability of seco by solving a relevant multi-phase rocket landing guidance problem with nonlinear dynamics and convex constraints only, and observe that our solver is 2.7 times faster than a state-of-the-art convex optimization solver.
Keywords: Real-time optimal control; convex optimization; rocket landing; guidance & control
1. INTRODUCTION
Rocket landing guidance can be considered to be the generalization of powered-descent guidance (PDG) to include the unpowered phase(s) of flight. Precision landing techniques for orbital rockets harness convex optimization for realtime trajectory generation (A¸cıkme¸se and Ploen, 2007;
Blackmore, 2016). One such method, known as lossless convexification, was the first convex optimization-based algorithm to compute a rocket landing guidance trajectory for a mid-flight large divert maneuver onboard the vehicle. Sequential convex programming (SCP) techniques for rocket landing have recently emerged as a way to handle more generalized nonconvexities in the dynamics, state constraints, environmental constraints, and path constraints (Mao et al. (2016)). Such SCP algorithms have been demonstrated to successfully solve a wide range of 6-DoF rocket landing problems (Szmuk et al. (2019);
Reynolds et al. (2020b); Szmuk et al. (2020)).
Pseudospectral methods have been developed in the last decade for solving multi-phase trajectory optimization
⋆This work was supported by NASA grant NNX17AH02A.
problems, particularly in aerospace applications. These methods typically parameterize the state time-history with Chebyshev and Legendre polynomials, operate on nonuniform time-grids with node points that are roots of these polynomials, and use linking conditions to tie together different phases of flight. A multi-phase Radau pseudospectral method was introduced in (Garrido and
Sagliano, 2021) for ascent and powered-descent guidance.
The SPARTAN software package described in (Sagliano et al., 2021) has been demonstrated to solve multi-phase problems arising in space applications. Further, relevant details showcasing pseudospectral approaches are provided in (Hwang and Ahn, 2022; Ma et al., 2019; Zhang and
Zhang, 2022). Despite their ability to solve multi-phase problems, pseudospectral methods are typically untenable for real-time implementations (Malyuta et al., 2019).
In this paper, we present sequential conic optimization (seco), a novel matrix-inverse-free paradigm for solving nonconvex optimal control problems in real-time, using which we formulate and solve a multi-phase rocket landing guidance problem in a unified manner. The guidance algorithm has the ability to perform mid-burn engine arXiv:2212.00375v2  [math.OC]  29 May 2023

switching after ignition. The primary features of the seco framework are as follows: (1) time-interval dilation, which allows for nonuniform time-grids and free-phasetransition-time; (2) a new formulation of compound statetriggered constraints—called single-crossing compound state-triggered constraints—that is convex, provided the trigger and constraint conditions are convex; (3) a new approach to handling artificial infeasibility, i.e., virtual state, that preserves the shapes of the constraint sets and does not alter the dynamics manifold; and (4) pipg (proportionalintegral projected gradient), a high-performance first-order solver that effectively exploits the structure of trajectory optimization problems, making it well-suited for embedded applications (Yu et al. (2020)). Further, we use an inversefree exact discretization method to generate dynamically feasible solutions. Despite the layers of apparent complexity described, the method that we present is implemented via a low-footprint codebase that is easy to verify and validate.
2. DYNAMICS
2.1 Time-interval dilation
The original nonlinear dynamics, governing the evolution of state x(t) ∈Rnx with control input u(t) ∈Rnu, over the entire time-horizon, are given by Equation (1).
˙x(t) = f(t, x(t), u(t)), t ∈[0, tf)
(1)
Now, we consider the dynamics in the sub-interval [tk, tk+1), where 0 < tk < tk+1 < tN = t−
f , k = 1 : N −1—where a:b denotes the range of integers between (and including)
integers a and b—and define an affine map, τk(t), as shown in Equation (2).
τk(t) :=
t −tk t−
k+1 −tk
∋τk(t) : [tk, tk+1) →[0, 1)
(2)
This mapping is referred to as time-interval dilation, as it normalizes the wall-clock time-interval to a known fixed interval—in our case [0, 1)—by either shrinking or expanding—and hence dilating—the original time-interval.
Next, we apply the derivative operator with respect to the dilated time τk, denoted by
◦□, to Equation (1), and invoke the chain-rule, as shown in Equation (3), where t ∈[tk, tk+1).
◦x(t) = d dτk x(t) = dt dτk d dt x(t) = dt dτk
˙x(t) =
t−
k+1−tk

sk
˙x(t)
= sk f(t, x(t), u(t)) := F(t, x(t), u(t), sk)
(3)
The multiplier in Equation (3), sk := t−
k+1−tk ∈R+, which is nothing but the length of the kth wall-clock time-interval, is termed the dilation factor. By treating a phase-based subset of sk, k = 1:N −1, i.e., such that k ⊆1:N −1, as decision variables and discretizing the system over them, we allow the optimizer to decide what the temporal spacing of discrete nodes should be in each phase rather than use a uniform temporal grid over the entire horizon. In the approach we propose, this is the key to enabling freetransition-time multi-phase trajectory optimization within a single-shot optimization framework, without requiring any mixed-integer or nonconvex constraints to handle the discrete temporal events/switching.
Although it is possible to allow each dilation factor to be an independent decision variable, we choose to partition the temporal grid based on the phases of flight, and evenly space the temporal nodes within each phase. This measure is taken to mitigate extreme inter-sample constraint violation, which tends to occur when fully adaptive grids are used.
The time-dilated dynamics given by Equation (3) will be used henceforth. Note that using the time-dilated dynamics given by Equation (3) in lieu of Equation (1) converts the original free-final-time optimal control problem to an equivalent fixed-final-time optimal control problem, with the effective horizon being [0, N −1).
2.2 Linearization
A convex approximation of the original nonconvex optimal control problem is obtained by linearizing the nonlinear dynamics (Equation (3))—which leads to a linear timevarying (LTV) system—and keeping the convex constraints intact. The state, control, and parameter constraint sets are assumed to belong to a set that has separable closed-form projection operations. We stress that most of the common constraints in trajectory optimization problems naturally fit this template (Malyuta et al., 2022). Nonconvex constraints can be handled using this framework too, by means of either linearization or convex approximation.
2.3 Discretization
We assume a first-order hold (FOH) on the control input signal throughout the horizon and a zero-order hold (ZOH) on the control input signal only during discrete switching events such as engine startup and downselection. Both of these control parameterizations have two key properties that make them attractive for optimal control applications: (1) inter-sample satisfaction of the convex control constraints is guaranteed (provided they are satisfied at the discrete temporal nodes), which is in contrast to pseudospectral methods; and, (2) the resulting conic subproblem has a sparsity pattern that is amenable to real-time implementation (Malyuta et al., 2019; Szmuk et al., 2020).
In the FOH case, the control profile is parameterized as shown in Equation (4), where t ∈[tk, tk+1) and τk ∈[0, 1), as given by Equation (2).
u(τk) = (1 −τk) uk + τk uk+1, k = 1:N −1
(4)
The LTV dynamics can now be written: (1) using the piecewise-affine control input parameterization given by
Equation (4); and, (2) in terms of deviations from the reference, as shown in Equation (5). The reference quantities are denoted by □, and ∆□denotes the deviation of a quantity from its reference, i.e., ∆□:= □−□, and
∆
◦x(τk) :=
◦x(τk)−F(τk, x(τk), u(τk), sk). The approximate nature of the equation is an artifact of linearization of the original nonlinear dynamics via truncation of the higherorder (≥2) terms in the Taylor series expansion.
∆
◦x(τk) ≈A(τk) ∆x(τk) + B(τk) (1 −τk) ∆uk
+ B(τk) τk ∆uk+1 + S(τk) ∆sk (5)
The state transition matrix (STM) associated with Equation (5), denoted by Φ(τk, 0), τk ∈[0, 1), satisfies the following matrix differential equation:
◦Φ(τk, 0) = A(τk) Φ(τk, 0), with Φ(0, 0) = Inx. The unique solution to Equation (5) is given by Equation (6) (Antsaklis and Michel, 2006; Malyuta et al., 2022).

∆x(τk) = Φ(τk, 0) ∆x(0) +
Z τk
0
Φ(τk, ζ) ·
(6)
· {B(ζ) (1 −τk) ∆uk + B(ζ) τk ∆uk+1 + S(ζ) ∆sk} dζ
Fig. 1. Propagation of the state. ∆x(t1):= 0, where t1 := 0.
The stitching condition for k = 1 : N −1 is given by
∆x
t−
k+1

+ x
t−
k+1

= ∆x(tk+1) + x(tk+1) = x(tk+1).
Evaluating Equation (6) at τk = 1−, we get Equation (7).
We replace the limits 0 and 1 with 0k and 1k, respectively, to explicitly indicate the dependence on index k, i.e.,
τk(tk) := 0k and τk
t−
k+1
 := 1−
k .
∆x
1−
k

= Ak∆x(0k) + B−
k ∆uk + B+
k ∆uk+1 + Sk∆sk (7)
Ak, B−
k , B+
k , and Sk can be computed as the solution to the initial value problem (IVP) given by Equations (8), respectively, integrated from 0k to 1−
k .
◦ΨA(ζ) = A(ζ) ΨA(ζ)
(8a)
◦ΨB−(ζ) = A(ζ) ΨB−(ζ) + B(ζ) (1 −ζ)
(8b)
◦ΨB+(ζ) = A(ζ) ΨB+(ζ) + B(ζ) ζ
(8c)
◦ΨS(ζ) = A(ζ) ΨS(ζ) + S(ζ)
(8d)
where function ΨA(ζ) is defined such that ζ 7→Φ(ζ, 0k), and the initial conditions for Equations (8) are: ΨA(0k) =
Inx, ΨB−(0k) = ΨB+(0k) = 0nx×nu, and ΨS(0k) = 0nx.
Note that Equation (9), which we refer to as the stitching condition, holds, as is evident from Figure 1.
∆x
1−
k

+ x
1−
k

= ∆x(1k) + x(1k)
(9)
The discretized dynamics can now be given by Equation (10), where ∆xk := ∆x(0k), xprop k+1 := x
1−
k

, xk+1 := x(1k), and u(τk) := (1 −τk) uk + τk uk+1, for 1 ≤k ≤N −1.
∆xk+1 = Ak∆xk + B−
k ∆uk + B+
k ∆uk+1 + Sk∆sk
+ xprop k+1 −xk+1
(10)
The discretized dynamics in terms of the absolute variables are recovered from Equation (10), as shown in Equation (11).
xk+1 = Ak xk + B−
k uk + B+
k uk+1 + Sk sk +
(11)
xprop k+1 −
Ak xk + B−
k uk + B+
k uk+1 + Sk sk

Equation (11) represents an exact discretization of the
LTV dynamics, which means that the error between the continuous-time trajectory and the discrete-time trajectory at the discrete temporal nodes is analytically zero.
In order to obtain the corresponding expressions for the ZOH case, the following changes are incorporated:
(1) Equation (8b) is replaced by Equation (12) (and accordingly, B−
k is replaced by Bk); and, (2) B+
k is set to a zero matrix with the same dimensions.
◦ΨB(ζ) = A(ζ) ΨB(ζ) + B(ζ)
(12)
3. STATE-TRIGGERED CONSTRAINTS
We introduce a specialized formulation of compound statetriggered constraints (STCs) for problems in which the trigger functions are activated only once and are strictly monotonic in a neighborhood around which they are activated. We refer to these constraints as single-crossing compound state-triggered constraints.
Let g(·) be a trigger function that is said to be activated on the set {x | g(x) ≤g⋆} for some trigger value g⋆. Further, let g−1(g⋆) be a well-defined pre-image. Then, g(·) is called a single-crossing trigger function if x⋆:= g−1(g⋆) is a singleton and g(·) is strictly monotonic in a neighborhood around x⋆. Such a formulation is especially useful in applications such as rocket landing that require certain
STCs to be satisfied for mission success, wherein it is reasonable to expect the trigger conditions to be activated once and only once. For instance, it is reasonable to expect/require a rocket in descent from a certain initial altitude, hi, with its target landing location at the origin, to certainly cross a trigger altitude 0 < htrigger < hi once during its descent, and not surpass that altitude after.
For the purpose of demonstration, we consider a compound
STC with one trigger condition and multiple constraint conditions to be imposed with the and logic. Let g(·) be the trigger function and cj(·), j = 1:nc, be the constraint functions, where nc is the number of constraint conditions.
The purpose of the compound STC (Szmuk et al., 2019,
Section II.B) is to satisfy the condition given in Equation (13), where x(t) is the state. ∀t ∈[0, tf), g(x(t)) ≤0 ⇒
nc
^
j=1
cj(x(t)) ≤0
(13)
In maneuvers with a single-crossing trigger condition, the trigger condition is activated once and only once, i.e., g(x(t)) = 0 is guaranteed to activate at some t =
ttrigger, g(x(t)) > 0 ∀t ∈[0, ttrigger), and g(x(t)) < 0
∀t ∈(ttrigger, tf). Using this fact, the single-crossing compound STC is formulated as shown in Equation (14).
We emphasize that ttrigger itself is free, and hence the compound constraint is state-triggered and not timetriggered.
g(x(t)) ≥0
∀t ∈[0, ttrigger)
(14a)
g(x(t)) = 0, t = ttrigger (14b)
g(x(t)) ≤0
c1(x(t)) ≤0
...
cnc(x(t)) ≤0







∀t ∈(ttrigger, tf)
(14c)
If the trigger conditions and the constraint conditions above are individually convex, the compound STC is entirely convex. This is in contrast to existing formulations of STCs in the literature that are inherently nonconvex, regardless of the convexity of the trigger and constraint conditions (Szmuk et al., 2019, 2020; Reynolds et al., 2020b). However, those methods are more general, in that they do not mandate the trigger condition to be single-crossing—we trade off this generality for convexity/simplicity in our method. Further, we note that the single-crossing STC
formulation bears resemblance to the method adopted in (Bhasin, 2016).

The aforementioned approaches in the literature use uniform temporal spacing of discrete nodes over the entire horizon. As a result, along with the fact that inter-sample constraint satisfaction is typically not guaranteed in general, these approaches do not guarantee the imposition of the constraints exactly at the specified trigger conditions. The triggering of these constraints (in time) is only accurate up to the spacing of the grid, and the corresponding solutions usually demonstrate violation of these constraints with respect to the triggers. This issue becomes more prevalent in maneuvers over very long time horizons, especially when the set of feasible trigger windows is much smaller in comparison, and can be detrimental to mission success if accurate triggering is required.
We leverage time-interval dilation to impose single-crossing
STCs, and treat the windows within which these constraints need to be imposed as distinct phases of flight. This allows for a fine grid in phases that involve critical constraints that need to be satisfied to ensure mission success, and a coarser grid in the more benign phases of flight—thus enabling one-shot multi-phase trajectory optimization. If the solution converges to a feasible trajectory, the STCs are guaranteed to be satisfied at the triggers (since the triggers are imposed as waypoints as in Equation (14b)), and at every discrete temporal node within the trigger window.
4. SEQUENTIAL CONIC OPTIMIZATION
4.1 Virtual state
Artificial infeasibility refers to the phenomenon wherein a subproblem can become infeasible as a result of linearization of the nonconvex constraints even if there exists a feasible solution to the original problem. Typically, unconstrained yet heavily penalized slack variables are added to the linearized constraints, so as to ensure that the subproblem is always feasible.
We propose a new approach to handling artificial infeasibility by means of a virtual state variable, which serves as a copy of the original state. This approach helps decouple the dynamics and control constraints from the state constraints and exactly satisfy all the path constraints at each solver iteration, while ensuring that the subproblem never turns infeasible. If x is the actual state variable, u is the control variable, and ξ is the virtual state variable, the dynamics constraint is imposed on x and u, the control constraints are imposed on u, and all the state constraints are imposed on ξ.
To ensure that the dynamics and all other constraints are satisfied at convergence, we minimize the error between x and ξ by heavily penalizing the squared distance between them in the objective function. The virtual state does not alter the dynamics manifold (unlike the virtual control approach (Szmuk et al., 2020; Reynolds et al., 2020b)), and preserves the shapes of the conic state constraint sets (unlike the virtual buffer approach (Malyuta et al., 2022)).
4.2 Conic subproblem
We impose a soft trust region on the decision variable and use the penalized trust region (ptr) algorithm (Szmuk et al., 2020; Reynolds et al., 2020a,b). The discretized conic subproblem with virtual state(s) and a soft trust region is shown in Problem (15), which is strongly convex.
min u, s wcJ(xN) + 1
2(wtrJtr + wvseJvse)
(15a)
s.t.
xk+1 = RHS of Eq. (11), k = 1:N −1
(15b)
ξk ∈X , k = 1:N
(15c)
uk ∈U , k = 1:N
(15d)
sk ∈S , k = 1:nphase (15e)
where x ∈Rnx is the state vector, ξ ∈Rnx is the virtual state vector, u ∈Rnu is the control input vector, and s ∈R++ is the dilation factor (vector); X , U , and S
are the state, control, and temporal constraint sets, respectively, which are assumed to be closed and convex; J(xN) is the original cost function, assumed to be in the Mayer form (Berkovitz, 2013), Jtr
:=
PN
k=1

∥xk −xk∥2
2 + ∥uk −uk∥2
2

+ Pnphase k=1
∥sk −sk∥2
2 is the trust region penalty, and Jvse := PN
k=1 ∥xk −ξk∥2
2 is the virtual state error penalty, wc, wtr, and wvse being their respective weights.
Proposition 1. The virtual state error penalty, given by
PN
k=1 ∥xk −ξk∥2
2, is convex.
Proof. Let yk := (xk, ξk). ∴∥xk −ξk∥2
2 = y⊤
k Myk, where
M =

1 −1
−1
1

⊗Inx. Since spec M ∈{0, 2}, M is positive semidefinite (PSD). Therefore, the quadratic form, y⊤
k Myk, is convex. Since the sum of convex functions is convex,
PN
k=1 y⊤
k Myk = PN
k=1 ∥xk −ξk∥2
2 is also convex.
■
Problem (15) can be vectorized, i.e., assembled into the form of Problem (16), by stacking all the decision variables into a single vector, z. For more details, see (Yu et al.,
2022a).
min z
1
2z⊤Qz + ⟨q, z⟩
(16a)
s.t.
Hz −h = 0
(16b)
z ∈D
(16c)
5. HIGH-PERFORMANCE SOLVER
The proportional-integral projected gradient method (pipg)
(Yu et al., 2020, 2022b) is a first-order primal-dual optimization algorithm for solving conic optimization problems such as the one given by Equations (16). pipg is compatible with extrapolation, which has been shown to improve its practical convergence behavior. Here, we use the extrapolated pipg algorithm, also denoted by xpipg, given by the iterative sequence in Equation (17) (Yu et al., 2022a).
zj+1 = πD

ζj −α
Qζj + q + H⊤ηj
(17a)
wj+1 = ηj + β
H
2zj+1 −ζj
−h

(17b)
ζj+1 = (1 −ρ)ζj + ρzj+1
(17c)
ηj+1 = (1 −ρ)ηj + ρwj+1
(17d)
where the primal variable, zj, converges to an optimum of Problem (16) as j
→∞. The step-sizes, α and
β, are computed in accordance with (Yu et al., 2022a,
Lemma 2), and ρ ∈[1, 2) is the extrapolation factor. We leverage the warm-starting capability of pipg and use the ℓ2-hypersphere preconditioning technique described in (Kamath et al., 2023) to further accelerate convergence.

6. MULTI-PHASE ROCKET LANDING GUIDANCE
As a representative example for multi-phase rocket landing guidance via seco, we consider the problem of terrestrial precision landing of a vehicle akin to Starship, which is designed to be a fully reusable rocket and currently in development (Shotwell and Blackmore, 2021).
We consider a nonlinear planar model of the vehicle, given by Equations (18), with ˆx being the vertical axis and ˆz being the horizontal axis in the inertial frame (and the longitudinal and lateral axes in the body frame, respectively).
We use a simple, tractable aerodynamic model to account for the aerodynamic effects on the vehicle during descent (Szmuk et al., 2020). In practice, however, numerical databases can be used along with seco (Mceowen and
A¸cıkme¸se, 2022). Further, with our approach, aerodynamicfree polynomial coast-phase ballistic trajectory predictions, such as the one provided in (Szmuk et al., 2020), are not required. We note that (Lee and Lee, 2022) provide an
SCP-based 6-DoF multi-phase rocket landing guidance implementation, but impose nonconvex STCs and implement inexact (trapezoidal) discretization, unlike our approach.
The original state vector is defined as follows: x(t) :=
(m(t), r(t), v(t), θ(t), ω(t)), where m(t) ∈R+ is the mass, r(t) ∈R2 is the position, v(t) ∈R2 is the velocity, θ(t) ∈R
is the body tilt angle with respect to the inertial vertical, and ω(t) ∈R is the angular velocity of the body.
The control input vector is defined as follows: u(t) :=
(T(t), δ(t)), where T(t) ∈R is the thrust magnitude and
δ(t) ∈R is the gimbal deflection angle.
Further, we define a virtual state vector and impose all the state constraints, including the boundary conditions, on this variable, in accordance with Subsection 4.1: ξ(t) :=
mξ(t), rξ(t), vξ(t), θξ(t), ωξ(t)

.
˙m(t) = −αeT(t)
(18a)
˙r(t) = v(t)
(18b)
˙v(t) =
1
m(t)(FI(t) + AI(t)) + g (18c)
˙θ(t) = ω(t)
(18d)
˙ω(t) =
1
J(t)
FBˆz(t) lcm −ABˆz(t) lj cp

(18e)
where
αe :=
1
Ispg0
(19a)
FI(t) := T(t)

cos(θ(t) + δ(t))
−sin(θ(t) + δ(t))

(19b)
AI(t) := RI←B(t)AB(t)
(19c)
FB(t) := T(t)

cos(δ(t))
−sin(δ(t))

(19d)
AB(t) := −ρairSarea∥v(t)∥2
2
CaeroR⊤
I←B(t)v(t)
(19e)
RI←B(t) :=

cos θ(t) sin θ(t)
−sin θ(t) cos θ(t)

(19f)
Here, αe ∈R++ is the thrust-specific fuel consumption (TSFC), Isp ∈R++ is the specific impulse of the rocket engine, g0 ∈R++ is standard Earth gravitational acceleration, g := (−g0, 0), RI←B(t) ∈SO(2) is the rotation matrix that maps coordinates in the body frame to the inertial frame, ρair ∈R++ is the ambient atmospheric density,
Sarea ∈R++ is the reference area, Caero := diag{cˆx, cˆz}
is the aerodynamic coefficient matrix, where cˆx, cˆz ∈R++
are the aerodynamic coefficients along the body ˆx and
ˆz axes, respectively, and J(t) ∈R++ is the moment of inertia of the vehicle about the body ˆy axis (out-of-plane).
The body of the vehicle is assumed to be a uniform solid cylinder, and hence, the moment of inertia about its central diameter is given by J(t) := m(t)

l2
r
4 + l2
h
12

, where lr ∈R++ and lh ∈R++ are the radius and height of the fuselage, respectively.
The engines are assumed to be co-located, and the location of the vehicle mass-center is assumed to be fixed in the body frame. The thrust moment-arm (the distance between the vehicle mass-center and the engine gimbal hinge point)
is denoted by lcm ∈R++, and lj cp ∈R+ is the aerodynamic moment-arm (the distance between the vehicle mass-center and the center-of-pressure), where j ∈{0, 1}; j = 0 for the unpowered phase of flight and j = 1 for the powered phases of flight—l0
cp is assumed to be maintained at zero via independent aerodynamic controls by means of forward and aft flaps (aerodynamic control surfaces), i.e., the center-ofpressure and the mass-center are assumed to be coincident when the vehicle is in the coast phase.
At the engine ignition (PDI) epoch, it is assumed that the following events occur: (1) the forward flaps are fully extended (to maximize drag towards the nose-cone); and, (2) the aft flaps are fully folded (to minimize drag towards the aft section of the vehicle). As a result, the center-ofpressure shifts away from the mass-center, towards the nose-cone, and induces an aerodynamic torque (pitching moment) on the vehicle. Hence, l1
cp is set to a nonzero value, and it is kept fixed for the remainder of the trajectory. This, along with gimbaling of the rocket engines, is used to induce the flip maneuver to get the vehicle upright in preparation for terminal descent.
The guidance problem is partitioned into four phases:
(1) the unpowered, subsonic coast phase; (2) the highthrust (3-engine) burn phase; (3) the low-thrust (1-engine)
burn phase; and, (4) the altitude-triggered terminal descent phase, separated by the following important discrete events/epochs: (1) powered-descent initiation (PDI) or engine ignition, tignition; (2) engine downselection or switching from a triple-engine burn to a single-engine burn, tswitch; and, (3) altitude-based triggering of the terminal descent phase, ttrigger. We emphasize that putting all of these phases together implicitly leads to a free-ignition-time, free-engineswitching-time, and free-final-time optimal control problem (subject to the temporal constraints imposed).
For the discretized problem, the temporal grid we choose is as given by Equation (20), where N is the number of discrete temporal nodes and kignition, kswitch, and ktrigger are the nodes at which the discrete events occur.
k ∈{1, . . . , kignition, . . . , kswitch, . . . , ktrigger, . . . , N} (20)
6.1 Common constraints
The dynamics and temporal constraints are imposed over the entire horizon. The dynamics constraint is given by
Equation (15b). The constraint given by Equation (21) is imposed on the dilation factors to ensure they are bounded.

−100
0
100
Downrange [m]
0
200
400
600
800
1000
Altitude [m]
htrigger
Glideslope
20
40
60
80
Speed [m/s]
3 engines
1 engine
0
2000
4000
6000
Thrust, T [kN]
3×Tmax
3×Tmin
Tmax
Tmin
−10
0
10
Gimbal angle, δ [◦]
+δmax
−δmax
85
90
95
100
Mass, m [103 kg]
mi mdry
−10
0
10
Gimbal rate, ˙δ [◦s−1]
+ ˙δmax
−˙δmax
0
500
1000
Altitude, rx [m]
Trigger (waypoint)
0
50
Speed, ∥v∥2 [ms−1]
vterminal vmax
0
5
10
15
20
Time [s]
0
50
Tilt angle, θ [◦]
+θmax
−θmax
0
5
10
15
20
Time [s]
−40
−20
0
Angular velocity, ω [◦s−1]
+ωmax
−ωmax tignition tswitch ttrigger
Solution
Single-shot integration
Interpolation
Fig. 2. A real-time multi-phase rocket landing guidance solution obtained via seco.
smin ≤sl ≤smax, l ∈{1:nphase}
(21)
In order to enable the imposition of a single-crossing compound STC in the altitude-triggered terminal descent phase, we impose a minimum altitude constraint in the first three phases, as given by Equation (22).
rξ
xk ≥htrigger, k ∈{1:ktrigger−1}
(22)
6.2 The unpowered coast phase, k ∈{1:kignition−1}
The center-of-pressure is coincident with the mass-center, i.e., j = 0 and lj cp = l0
cp = 0 in the dynamics. In implementation, the thrust magnitude is set to zero (the gimbal angle is inconsequential), and a zero-order hold (ZOH) is assumed on the control input signal for this phase, in order to avoid control input constraint violation when the engines are ignited in the next phase. A first-order hold (FOH) can also be assumed here, if the engine startup time is significant and needs to be taken into account (with appropriate constraints on the dilation factor).
The following initial conditions are imposed:
mξ
1 = mi, rξ
1 = ri, vξ
1 = vi,
θξ
1 = θi, ωξ
1 = ωi, T1 = 0
(23)
where mi, ri, vi, θi, and ωi are the initial values for the mass, position, velocity, body tilt angle, and angular velocity, respectively. The thrust magnitude at the first node is constrained to be zero.
6.3 The high-thrust burn phase, k ∈{kignition :kswitch−1}
The center-of-pressure shifts towards the nose-cone of the vehicle, i.e., j = 1 and lj cp = l1
cp > 0 in the dynamics.
We model this as a discrete change in its value, which is then held constant for the remainder of the trajectory. The following constraints are imposed on the control variables:
3 Tmin ≤Tk ≤3 Tmax (24)
max{−δmax, −˙δmax¯sk−1 + ¯δk−1} ≤δk
≤min{δmax, ˙δmax¯sk−1 + ¯δk−1}
(25)
where Tmin and Tmax are the lower and upper bounds on the thrust magnitude for a single engine, respectively,
−δmax and δmax are the lower and upper bounds on the gimbal deflection angle, respectively, and −˙δmax and
˙δmax are the lower and upper bounds on the gimbal rate, respectively. Equation (25) combines the gimbal angle and rate constraints, by using reference values in the lower and upper bounds to make them constants and hence, avoid overlapping projections on the same variable. This constraint is, however, exact at convergence, and this formulation has been observed to work well in practice.
Although there are only nphase dilation factor decision variables, we consider the length of ¯s to be N−1, such that the dilation factors are repeated to span each phase.
ZOH is assumed on the control input signal between kswitch−1 and kswitch, in preparation for engine downselection, which marks the beginning of the next phase.
FOH can also be assumed here, if the engine shutdown time is significant and needs to be accounted for. In order to ensure that the gimbal deflection angle profile does not have any discontinuities, we set the gimbal rate to zero between these nodes, as shown in Equation (26).
δkswitch = ¯δkswitch−1
(26)

6.4 The low-thrust burn phase, k ∈{kswitch :ktrigger−1}
This phase is similar to the high-thrust burn phase, apart from the fact that the switch from 3 to 1 engines occurs at kswitch, and the bounds on the thrust magnitude are changed accordingly, as shown in Equation (27). The combined gimbal constraint is left unchanged from the previous phase, and is shown in Equation (28). FOH is assumed on the control input signal in this phase, and for the remainder of the trajectory.
Tmin ≤Tk ≤Tmax (27)
max{−δmax, −˙δmax¯sk−1 + ¯δk−1} ≤δk
≤min{δmax, ˙δmax¯sk−1 + ¯δk−1}
(28)
6.5 The terminal descent phase, k ∈{ktrigger :N}
The final phase, the terminal descent phase, is the most heavily and tightly constrained phase of flight. This is designed as such in order to enable closed-loop precision landing, i.e, to ensure that the generated guidance trajectories (the feedforward control input signal and the reference state profiles) are amenable to tight tracking via feedback controllers. Such a phase would be especially useful if sub-meter touchdown accuracy is required, for instance, if the vehicle is to be retrieved by the launch tower itself (Atkinson, 2022).
The constraint on the thrust magnitude is left unchanged from the previous phase, and is shown in Equation (29).
Tmin ≤Tk ≤Tmax (29)
An altitude-triggered single-crossing compound STC, with five constraint conditions, is imposed, as shown in Equations (30). The constraint conditions include the following:
maximum speed, maximum tilt, maximum angular speed, glideslope, and tighter gimbal deflection bounds. The altitude constraint forms the trigger condition. All of these constraints are imposed in the interval k ∈{ktrigger :N−1}.
rξ
xk
= htrigger, if k = ktrigger
≤htrigger, otherwise (30a)
|rξ
zk| ≤
 tan γgs htrigger, if k = ktrigger tan γgs ¯rξ
xk, otherwise (30b)
∥vξ
k∥2 ≤vmax (30c)
|θξ
k| ≤θmax (30d)
|ωξ
k| ≤ωmax (30e)
max{−δmaxTD, −˙δmax¯sk−1 + ¯δk−1} ≤δk (30f)
≤min{δmaxTD, ˙δmax¯sk−1 + ¯δk−1}
The altitude and glideslope constraints are treated differently at the trigger epoch and after. The altitude constraint is posed as an equality at the trigger—this ensures that the constraint conditions are exactly satisfied at the trigger.
Further, the glideslope constraint is cast in the form of box constraints in terms of the reference values, in order to enable closed-form projections (without this measure, there would be two constraints on rξ
xk at every temporal node, thus precluding closed-form projections). Similar to the combined gimbal constraint, this constraint is exact at convergence. The bounds on the gimbal deflection angle are tightened in this phase as well, i.e., δmaxTD < δmax.
The following terminal boundary conditions are imposed:
mξ
N ≥mdry, rξ
N = rf, vξ
N = vf,
θξ
N = θf, ωξ
N = ωf, δN = 0
(31)
where mdry is the dry mass of the vehicle, and rf, vf, θf, and ωf are the terminal values for the position, velocity, body tilt angle, and angular velocity, respectively. The gimbal angle at the final node is constrained to be zero to avoid plume-impingement on the retrieval structure.
6.6 The discrete seco subproblem
We define J(xN) in Equation (15a) to be −mN, i.e., the final mass of the vehicle is maximized (thus minimizing propellant consumption). The discrete seco subproblem, which is a second-order cone program (SOCP), can now be given as follows:
min
Objective function: Equation (15a)
s.t.
Common constraints, Subsection 6.1:
Equations (15b), (21), and (22)
Coast phase, Subsection 6.2:
Equations (23)
High-thrust burn phase, Subsection 6.3:
Equations (24), (25), and (26)
Low-thrust burn phase, Subsection 6.4:
Equations (27) and (28)
Terminal descent phase, Subsection 6.5:
Equations (29), (30), and (31)
7. NUMERICAL RESULTS
For our numerical implementation of the multi-phase rocket landing guidance algorithm, we choose a grid of N = 16
discrete temporal nodes, with one node allocated to the unpowered coast phase, and 5 nodes allocated to each of the remaining three phases of flight, i.e, kignition = 2, kswitch = 7, ktrigger = 12. The following parameters are used, where the ones pertaining to the vehicle/maneuver were either estimated or obtained from public sources (DeSisto, 2021; Sagliano et al., 2021; Malyuta et al., 2022).
g0 = 9.81 ms−2, Isp = 330 s, lr = 4.5 m, lh = 50 m, lcm = 0.4 lh, l0
cp = 0 m, l1
cp = 0.2 lh, ρair = 1.225 kgm−3,
Sarea = 545 m2, vterminal = 85 ms−1, mi = 100000 kg, cˆx = 0.0522, cˆz = 0.4068, Tmax = 2200 kN, Tmin = 880 kN,
δmax = 10◦, ˙δmax = 15◦s−1, htrigger = 100 m, γgs = 5◦, vmax = 20 ms−1, θmax = 5◦, ωmax = 2.5◦s−1, δmaxTD = 1◦, mdry = 85000 kg, ri = (1000, 100) m, vi = (−90, 0) ms−1,
θi = 90◦, ωi = 0◦s−1, rf = (0, 0) m, vf = (0, 0) ms−1,
θf = 0◦, ωf = 0◦s−1, smin = 0.6 s, smax = 10 s
Averaged over 100 full seco solves, we report a mean runtime of the pipg solver (to solve the entire nonconvex problem) of 13.7 ms. In comparison, ecos1 requires 37.1
ms, on average, to solve the problem. For the comparison, we assess the quality of the converged solutions in terms of the difference in propellant consumption (0.02%) and the number of seco iterations required to converge (7).
A real-time guidance solution obtained via pipg is shown
1For this problem, ecos is faster than both mosek and gurobi.

in Figure 2. Here, we observe that the optimizer chooses to initiate the powered-descent phase at an altitude of
490.34 m and a speed of 86.28 ms−1 (which is very close to the terminal velocity). Further, we note that the numerous prototype flight tests and independent analyses corroborate many of our observations (DeSisto, 2021).
8. CONCLUSIONS
We introduce seco, a novel matrix-inverse-free paradigm for solving nonconvex optimal control problems in real-time, and solve a multi-phase rocket landing guidance problem with free-transition-time and convex state-triggered constraints. These solutions are computed using pipg, which is almost three times faster than ecos, a state-of-the-art convex optimization solver.
ACKNOWLEDGEMENTS
The authors thank the members of the Autonomous Controls Laboratory, especially Taewan Kim, Dayou Luo, and
Samet Uzun, and also the members of the Flight Mechanics and Trajectory Design branch (EG5) at the NASA Johnson
Space Center, especially Gavin Mendeck, for their valuable insight and the many helpful discussions. The authors also give their special thanks to Miki Szmuk, Taylor Reynolds, and Danylo Malyuta, for laying the foundation for much of this research and also their continued support. This research was supported by NASA grant NNX17AH02A
and was partially carried out at the NASA Johnson Space
Center; Government sponsorship is acknowledged.
REFERENCES
A¸cıkme¸se, B. and Ploen, S.R. (2007). Convex programming approach to powered descent guidance for Mars landing.
AIAA Journal of Guidance, Control and Dynamics,
30(5), 1353–1366.
Antsaklis, P.J. and Michel, A.N. (2006). Linear systems.
Basel, Switzerland: Birkhauser.
Atkinson, N. (2022). SpaceX tests its Starship-catching launch tower. www.universetoday.com/spacex-tests-itsstarship-catching-launch-tower. Accessed: 11.18.2022.
Berkovitz, L.D. (2013). Optimal control theory, volume 12.
Springer Science & Business Media.
Bhasin, N.S. (2016). Fuel-optimal spacecraft guidance for landing in planetary pits.
Master’s thesis, Carnegie
Mellon University, Pittsburgh, PA.
Blackmore, L. (2016). Autonomous precision landing of space rockets. In in Frontiers of Engineering: Reports on Leading-Edge Engineering from the 2016 Symposium, volume 46, 15–20.
DeSisto, A. (2021).
Starship and its belly flop maneuver. www.everydayastronaut.com/starships-belly-flopmaneuver. Accessed: 10.14.2022.
Garrido, J.V. and Sagliano, M. (2021).
Ascent and descent guidance of multistage rockets via pseudospectral methods. In AIAA SciTech 2021 Forum.
Hwang, J. and Ahn, J. (2022). Integrated optimal guidance for reentry and landing of a rocket using multi-phase pseudo-spectral convex optimization.
International
Journal of Aeronautical and Space Sciences, 1–9.
Kamath, A.G., Elango, P., Kim, T., Mceowen, S., Yu, Y.,
Carson III, J.M., Mesbahi, M., and A¸cıkme¸se, B. (2023).
Customized real-time first-order methods for onboard dual quaternion-based 6-DoF powered-descent guidance.
In AIAA SciTech 2023 Forum.
Lee, S.D. and Lee, C.H. (2022). Multi-phase and dual aero/propulsive rocket landing guidance using successive convex programming. Proceedings of the Institution of
Mechanical Engineers, Journal of Aerospace Engineering.
Ma, L., Wang, K., Shao, Z., Song, Z., and Biegler, L.T.
(2019). Direct trajectory optimization framework for vertical takeoff and vertical landing reusable rockets: case study of two-stage rockets. Engineering Optimization.
Malyuta, D., Reynolds, T., Szmuk, M., Mesbahi, M.,
A¸cıkme¸se, B., and Carson, J.M. (2019). Discretization performance and accuracy analysis for the rocket powered descent guidance problem. In AIAA Scitech 2019 Forum.
Malyuta, D., Reynolds, T.P., Szmuk, M., Lew, T., Bonalli,
R., Pavone, M., and A¸cıkme¸se, B. (2022).
Convex optimization for trajectory generation: A tutorial on generating dynamically feasible trajectories reliably and efficiently. IEEE Control Systems, 42(5), 40–113.
Mao, Y., Szmuk, M., and A¸cıkme¸se, B. (2016). Successive convexification of non-convex optimal control problems and its convergence properties.
In 2016 IEEE 55th
Conference on Decision and Control (CDC), 3636–3641.
Mceowen, S. and A¸cıkme¸se, B. (2022). Hypersonic entry trajectory optimization via successive convexification with abstracted control. In AIAA SciTech 2022 Forum.
Reynolds, T., Malyuta, D., Mesbahi, M., A¸cıkme¸se, B., and Carson, J.M. (2020a). A real-time algorithm for non-convex powered descent guidance. In AIAA Scitech
2020 Forum.
Reynolds, T.P., Szmuk, M., Malyuta, D., Mesbahi, M.,
A¸cıkme¸se, B., and Carson III, J.M. (2020b).
Dual quaternion-based powered descent guidance with statetriggered constraints. Journal of Guidance, Control, and
Dynamics.
Sagliano, M., Seelbinder, D., and Theil, S. (2021). SPARTAN: Rapid trajectory analysis via pseudospectral methods. In ICATT.
Shotwell, G.E. and Blackmore, L. (2021). Space launch in 50 years: Abundance at last? National Academy of
Engineering.
Szmuk, M., Reynolds, T., A¸cıkme¸se, B., Mesbahi, M., and
Carson, J.M. (2019). Successive convexification for 6-DoF
powered descent guidance with compound state-triggered constraints. In AIAA Scitech 2019 Forum.
Szmuk, M., Reynolds, T.P., and A¸cıkme¸se, B. (2020).
Successive convexification for real-time six-degree-offreedom powered descent guidance with state-triggered constraints. Journal of Guidance, Control, and Dynamics, 43(8), 1399–1413.
Yu, Y., Elango, P., and A¸cıkme¸se, B. (2020). Proportionalintegral projected gradient method for model predictive control. IEEE Control Systems Letters, 5(6), 2174–2179.
Yu, Y., Elango, P., A¸cıkme¸se, B., and Topcu, U. (2022a).
Extrapolated proportional-integral projected gradient method for conic optimization. IEEE Control Systems
Letters, 7, 73–78.
Yu, Y., Elango, P., Topcu, U., and A¸cıkme¸se, B. (2022b).
Proportional–integral projected gradient method for conic optimization. Automatica, 142, 110359.
Zhang, D. and Zhang, Y. (2022).
PySCP: A multiplephase optimal control software using sequential convex programming. International Journal of Aerospace Engineering, 2022.