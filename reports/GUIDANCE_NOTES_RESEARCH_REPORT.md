# Research Report: Convex / SOCP Powered-Descent Guidance — Extraction for a kOS Landing Script

**Scope.** Four Markdown extractions of PDFs were read in full and mined for quantitative, implementable findings.

| # | File | Size | Content type | Usable for kOS? |
|---|---|---|---|---|
| 1 | `2018_序列凸化六自由度火星火箭动力着陆_自由末时间.md` | 44,777 B / 1,157 lines | Full 15-page arXiv paper (Szmuk & Açıkmeşe, arXiv:1802.03827v1) | Mostly **(c)** offline insight, some **(b)** |
| 2 | `2022_多阶段火箭着陆序列锥优化实时求解SeCO.md` | 44,738 B / 673 lines | Full 8-page arXiv paper (Kamath et al., arXiv:2212.00375v2, SeCO) | Mostly **(c)**, some **(b)** |
| 3 | `2025_TUM硕士论文_多发动机可复用运载器容错着陆制导设计_SCP框架.md` | 183,574 B / 5,940 lines | Full 114-page TUM Master's thesis (Soldati, Oct 2025) | Mostly **(c)**, several **(b)** |
| 4 | `2012_最小着陆误差动力下降制导_行星着陆任务.md` | 5,064 B / 63 lines | **1-page NASA Tech Briefs blurb — NOT a paper** | **(c)** only, near-zero content |

**Tooling caveat (verified, not speculation).** Files #2 and #3 contain stray NUL (0x00) bytes from PDF extraction (14 and 18 respectively). The harness `read` tool rejects them as binary. I confirmed every NUL is a whitespace / page-gutter artifact (they sit around page footers such as `7 of 15`) with **no content lost**, and read them via NUL-stripped copies. File #1 also contains 20 NULs but `read` accepted it. Anyone re-reading should expect this.

**Headline answer up front.** None of these four documents contains anything that can be run directly on kOS. Every formulation in them requires an SOCP solver, matrix exponentials / state-transition matrices, or finite-difference Jacobians. The genuinely usable content for a 200–2000-op/frame interpreted script is: (a) the **thrust-structure theorem** (bang-bang / max-then-min, §3 below) which justifies a trivial 2-parameter control law; (b) the **transversality condition $H(t_f)=0$** as an offline check on $t_f$; (c) the **glide-slope constraint in its $x\tan\gamma \ge \|[\cdot]\|$ form**, which is directly computable; and (d) a set of **gain-scheduling / LQR tracking insights**. Details and the explicit reduction path are in §7.

---

## 1. The exact SOCP problem formulation

### 1.1 File #1 — Szmuk & Açıkmeşe 2018, 6-DoF minimum-time free-final-time

This is the most complete formulation of the four. Note carefully: **the objective is minimum TIME, not minimum fuel.**

#### State and control

> `2018_序列凸化六自由度火星火箭动力着陆_自由末时间.md`, Eq. 13a–13b

$$x(t) \triangleq \begin{bmatrix} m(t) & r_I^T(t) & v_I^T(t) & q_{B/I}^T(t) & \omega_B^T(t) \end{bmatrix}^T \in \mathbb{R}^{14}, \qquad u(t) \triangleq T_B(t) \in \mathbb{R}^3$$

State dimension $n_x = 14$ (1 mass + 3 position + 3 velocity + **4 quaternion** + 3 angular rate). Control dimension $n_u = 3$ (the body-frame thrust **vector**, not magnitude-plus-direction).

#### Dynamics and kinematics (Eqs. 1–5)

Mass depletion with the TSFC constant defined as $\alpha_{\dot m} \triangleq \dfrac{1}{I_{sp}g_0}$ (Eq. between 157–161):

$$\dot m(t) = -\alpha_{\dot m}\|T_B(t)\|_2 \tag{1}$$

$$\dot r_I(t) = v_I(t) \tag{2a}$$
$$\dot v_I(t) = \frac{1}{m(t)}F_I(t) + g_I \tag{2b}$$

Quaternion kinematics with the leading-scalar convention $q_{B/I} = [\cos(\xi/2),\ \sin(\xi/2)\hat n]^T$ (Eq. 3a–3b):

$$\dot q_{B/I}(t) = \tfrac{1}{2}\Omega\big(\omega_B(t)\big)q_{B/I}(t) \tag{3a}$$
$$J_B\dot\omega_B(t) = M_B(t) - [\omega_B(t)\times]J_B\omega_B(t) \tag{3b}$$

with the DCM from quaternion (Eq. after 186):

$$C_{B/I} = \begin{bmatrix} 1-2(q_2^2+q_3^2) & 2(q_1q_2+q_0q_3) & 2(q_1q_3-q_0q_2)\\ 2(q_1q_2-q_0q_3) & 1-2(q_1^2+q_3^2) & 2(q_2q_3+q_0q_1)\\ 2(q_1q_3+q_0q_2) & 2(q_2q_3-q_0q_1) & 1-2(q_1^2+q_2^2)\end{bmatrix}$$

Force/torque from thrust (Eqs. 4–5):

$$F_I(t) = C_{I/B}(t)T_B(t) \tag{4}, \qquad M_B(t) = [r_{T,B}\times]T_B(t) \tag{5}$$

Single gimbaled engine; $r_{T,B}$ is the constant CoM-to-gimbal moment arm. Aerodynamics assumed negligible.

#### State and control constraints (Eqs. 6–12)

$$m_{dry} \le m(t) \tag{6}$$

Glide-slope cone (Eq. 7a–7b) — **upward-facing cone, angle $\gamma_{gs}\in[0°,90°)$ from horizontal, centered at the origin**:

$$e_1\cdot r_I(t) \ge \tan\gamma_{gs}\,\|H_{23}^T r_I(t)\|_2 \tag{7a}, \qquad H_{23} \triangleq [e_2 \ \ e_3] \tag{7b}$$

Tilt angle (Eq. 8), defined as the angle between the $X$-axes of $F_B$ and $F_I$:

$$\cos\theta(t) = e_1\cdot C_{I/B}(t)e_1 = 1 - 2\big(q_2^2(t)+q_3^2(t)\big) \tag{8}$$

**Tilt constraint, convex (Eq. 9)** — this is the key trick for question 2:

$$\cos\theta_{\max} \le 1 - 2\big(q_2^2(t) + q_3^2(t)\big) \tag{9}$$

Angular-rate (Eq. 10): $\|\omega_B(t)\|_2 \le \omega_{\max}$

Thrust magnitude and gimbal (Eqs. 11–12), with $\delta_{\max}\in(0°,90°)$:

$$0 < T_{\min} \le \|T_B(t)\|_2 \le T_{\max} \tag{11}$$
$$\cos\delta_{\max}\|T_B(t)\|_2 \le e_1\cdot T_B(t) \tag{12}$$

The paper states explicitly (line 306): *"the upper thrust magnitude bound is a convex constraint, whereas the lower thrust magnitude bound is non-convex. For the range of $\delta_{\max}$ considered, the gimbal angle constraint is convex."*

> **Important nuance for the user's assumption.** This 2018 paper does **NOT** use the lossless-convexification $\sigma$ slack variable. It handles the non-convex $T_{\min}$ bound by **successive linearization** (Eqs. 18–19). The $\sigma$ (and log-mass) device is referenced only as prior work (refs. 13–20, notably Açıkmeşe & Ploen 2007 and Açıkmeşe, Carson & Blackmore 2013). If the user wants the actual $\sigma$ + log-mass formulation, **it is not in this file** — see §1.4.

#### Boundary conditions

Initial: $m(0)=m_{wet}$, $r_I(0)=r_{I,i}$, $v_I(0)=v_{I,i}$, $\omega_B(0)=\omega_{B,i}$ — **initial attitude is FREE**.
Final: $r_I(t_f)=0$, $v_I(t_f)=v_{I,f}$, $q_{B/I}(t_f)=q_{B/I,f}$, $\omega_B(t_f)=0$ — **final mass is FREE**.
Plus $e_2\cdot T_B(t_f) = e_3\cdot T_B(t_f) = 0$ (final thrust along the body X-axis, to null terminal torque).

#### The core SOCP: Problem 2 (lines 633–681)

**Cost:**
$$\minimize_{\sigma^i, u_k^i}\quad \sigma^i + w_\nu\|\bar\nu^i\|_1 + w_\Delta^i\|\bar\Delta^i\|_2 + w_{\Delta\sigma}\|\Delta_\sigma\|_1$$

**Boundary conditions:** $m_0^i = m_{wet}$; $r_{I,0}^i = r_{I,i}$; $r_{I,K}^i = 0$; $v_{I,0}^i = v_{I,i}$; $v_{I,K}^i = v_{I,f}$; $q_{B/I,K}^i = q_{B/I,f}$; $\omega_{B,0}^i = \omega_{B,i}$; $\omega_{B,K}^i = 0$; $e_2\cdot u_K^i = e_3\cdot u_K^i = 0$.

**Dynamics (the linearized, discretized recurrence, Eq. 26):**
$$x_{k+1}^i = \bar A_k^i x_k^i + \bar B_k^i u_k^i + \bar C_k^i u_{k+1}^i + \bar\Sigma_k^i\sigma^i + \bar z_k^i + \nu_k^i, \qquad \forall k\in\bar{\mathcal K}$$

**State constraints:**
$$m_{dry}\le m_k^i, \quad \tan\gamma_{gs}\|H_{23}r_{I,k}^i\|_2 \le e_1\cdot r_{I,k}^i, \quad \cos\theta_{\max}\le 1 - 2\|H_q q_{B/I,k}^i\|_2^2, \quad \|\omega_{B,k}^i\|_2\le\omega_{\max}$$

**Control constraints:**
$$T_{\min} \le B_g(\tau_k)u_k^i, \qquad \|u_k^i\|_2\le T_{\max}, \qquad \cos\delta_{\max}\|u_k^i\|_2 \le e_1\cdot u_k$$

**Trust regions:**
$$\delta x_k^i\cdot\delta x_k^i + \delta u_k^i\cdot\delta u_k^i \le \Delta_k^i, \qquad \|\delta\sigma^i\|_1 \le \Delta_\sigma^i$$

#### Linearization and discretization machinery

Normalized time $\tau\in[0,1]$ via the **dilation coefficient** (Eq. 15):
$$\sigma \triangleq \left(\frac{d\tau}{dt}\right)^{-1} \tag{15}$$

$$x'(\tau) = A(\tau)x(\tau) + B(\tau)u(\tau) + \Sigma(\tau)\sigma + z(\tau) \tag{17a}$$
$$A(\tau) \triangleq \hat\sigma\cdot\left.\frac{\partial}{\partial x}f(x,u)\right|_{\hat x(\tau),\hat u(\tau)} \tag{17b}$$
$$B(\tau) \triangleq \hat\sigma\cdot\left.\frac{\partial}{\partial u}f(x,u)\right|_{\hat x(\tau),\hat u(\tau)} \tag{17c}$$
$$\Sigma(\tau) \triangleq f\big(\hat x(\tau),\hat u(\tau)\big) \tag{17d}$$
$$z(\tau) \triangleq -A(\tau)\hat x(\tau) - B(\tau)\hat u(\tau) \tag{17e}$$

**Non-convex $T_{\min}$ handled by first-order Taylor linearization** (Eqs. 18–19b):
$$g\big(u(\tau)\big) \triangleq T_{\min} - \|u(\tau)\|_2 \le 0 \tag{18}$$
$$T_{\min} \le B_g(\tau)u(\tau) \tag{19a}, \qquad B_g(\tau) \triangleq \frac{\hat u^T(\tau)}{\|\hat u(\tau)\|_2} \tag{19b}$$

Discretization: $K$ evenly distributed points, $\tau_k \triangleq \frac{k}{K-1}$; **first-order hold (FOH) on the control** (Eq. 20a–20c):
$$u(\tau) \triangleq \alpha_k(\tau)u_k + \beta_k(\tau)u_{k+1},\quad \alpha_k(\tau)\triangleq\frac{\tau_{k+1}-\tau}{\tau_{k+1}-\tau_k},\quad \beta_k(\tau)\triangleq\frac{\tau-\tau_k}{\tau_{k+1}-\tau_k}$$

Discrete matrices (Eqs. 22b–22f) require the **state transition matrix** $\Phi_A$ solving $\frac{d}{d\tau}\Phi_A(\tau,\tau_k) = A(\tau)\Phi_A(\tau,\tau_k)$, $\Phi_A(\tau_k,\tau_k)=I$:
$$\bar A_k \triangleq \Phi_A(\tau_{k+1},\tau_k),\quad \bar B_k \triangleq \int_{\tau_k}^{\tau_{k+1}}\Phi_A(\tau_{k+1},\xi)B(\xi)\alpha_k(\xi)d\xi,\quad \bar C_k \triangleq \int_{\tau_k}^{\tau_{k+1}}\Phi_A(\tau_{k+1},\xi)B(\xi)\beta_k(\xi)d\xi$$
$$\bar\Sigma_k \triangleq \int_{\tau_k}^{\tau_{k+1}}\Phi_A(\tau_{k+1},\xi)\Sigma(\xi)d\xi,\qquad \bar z_k \triangleq \int_{\tau_k}^{\tau_{k+1}}\Phi_A(\tau_{k+1},\xi)z(\xi)d\xi$$

> The paper's stated advantage (line 65): it *"computes the discrete linear-time-variant system matrices to ensure that the converged solution perfectly satisfies the original nonlinear dynamics."* Note the contrast with file #2, which achieves exactness differently.

### 1.2 File #2 — Kamath et al. 2022/2023, SeCO multi-phase

#### State and control (planar / 3-DoF, nonlinear)

> `2022_多阶段火箭着陆序列锥优化实时求解SeCO.md`

Original state: $x(t) := \big(m(t), r(t), v(t), \theta(t), \omega(t)\big)$, $m\in\mathbb R_+$, $r\in\mathbb R^2$, $v\in\mathbb R^2$, $\theta\in\mathbb R$ (body tilt w.r.t. inertial vertical), $\omega\in\mathbb R$.
Control: $u(t) := \big(T(t), \delta(t)\big)$ — **thrust magnitude $T$ and gimbal deflection $\delta$** (separated, unlike file #1's vector control).
Virtual state: $\xi(t) := \big(m_\xi(t), r_\xi(t), v_\xi(t), \theta_\xi(t), \omega_\xi(t)\big)$.

#### Dynamics (Eqs. 18–19f)

$$\dot m = -\alpha_e T(t) \tag{18a}, \qquad \dot r = v \tag{18b}, \qquad \dot v = \frac{1}{m}(F_I + A_I) + g \tag{18c}, \qquad \dot\theta = \omega \tag{18d}, \qquad \dot\omega = \frac{1}{J(t)}\big(F_{B\hat z}l_{cm} - A_{B\hat z}l_{cp}^j\big) \tag{18e}$$

$$\alpha_e := \frac{1}{I_{sp}g_0} \tag{19a}, \quad F_I(t) := T(t)\begin{bmatrix}\cos(\theta+\delta)\\-\sin(\theta+\delta)\end{bmatrix} \tag{19b}, \quad A_I := R_{I\leftarrow B}A_B \tag{19c}$$
$$F_B(t) := T(t)\begin{bmatrix}\cos\delta\\-\sin\delta\end{bmatrix} \tag{19d}, \quad A_B(t) := -\rho_{air}S_{area}\|v\|_2^2 C_{aero}R_{I\leftarrow B}^Tv \tag{19e}, \quad R_{I\leftarrow B} := \begin{bmatrix}\cos\theta & \sin\theta\\-\sin\theta & \cos\theta\end{bmatrix} \tag{19f}$$

$J(t) := m(t)\big(\frac{l_r^2}{4} + \frac{l_h^2}{12}\big)$ (uniform solid cylinder, moment about central diameter).

#### Time-interval dilation (Eqs. 2–3) — different from file #1's $\sigma$

$$\tau_k(t) := \frac{t-t_k}{t_{k+1}^- - t_k}: [t_k, t_{k+1}) \to [0,1) \tag{2}$$
$${}^\circ x(t) = \frac{d}{d\tau_k}x(t) = \left(t_{k+1}^- - t_k\right)\dot x(t) = s_k f(t,x,u) := F(t,x,u,s_k) \tag{3}$$

The dilation factor $s_k := t_{k+1}^- - t_k\in\mathbb R_+$ **is the wall-clock length of interval $k$**, treated as a decision variable. Key stated benefit (line 94): *"using the time-dilated dynamics given by Equation (3) in lieu of Equation (1) converts the original free-final-time optimal control problem to an equivalent fixed-final-time optimal control problem, with the effective horizon being $[0,N-1)$."*

> This is the cleanest free-final-time mechanism across all four files, and it is **conceptually** reusable on kOS: reparametrize by node index instead of wall-clock time, and let interval lengths be free.

#### Exact discretization (Eqs. 7–11) — this is SeCO's party trick

$$\Delta x(1_k^-) = A_k\Delta x(0_k) + B_k^-\Delta u_k + B_k^+\Delta u_{k+1} + S_k\Delta s_k \tag{7}$$

$${}^\circ\Psi_A(\zeta) = A(\zeta)\Psi_A(\zeta) \tag{8a}, \quad {}^\circ\Psi_{B^-}(\zeta) = A(\zeta)\Psi_{B^-}(\zeta) + B(\zeta)(1-\zeta) \tag{8b}$$
$${}^\circ\Psi_{B^+}(\zeta) = A(\zeta)\Psi_{B^+}(\zeta) + B(\zeta)\zeta \tag{8c}, \quad {}^\circ\Psi_S(\zeta) = A(\zeta)\Psi_S(\zeta) + S(\zeta) \tag{8d}$$

with $\Psi_A(0_k)=I_{n_x}$, $\Psi_{B^-}(0_k)=\Psi_{B^+}(0_k)=0_{n_x\times n_u}$, $\Psi_S(0_k)=0_{n_x}$.

$$\Delta x_{k+1} = A_k\Delta x_k + B_k^-\Delta u_k + B_k^+\Delta u_{k+1} + S_k\Delta s_k + x^{prop}_{k+1} - x_{k+1} \tag{10}$$

$$x_{k+1} = A_kx_k + B_k^-u_k + B_k^+u_{k+1} + S_ks_k + \Big(x^{prop}_{k+1} - \big(A_kx_k + B_k^-u_k + B_k^+u_{k+1} + S_ks_k\big)\Big) \tag{11}$$

The paper states (line 190): *"Equation (11) represents an exact discretization of the LTV dynamics, which means that the error between the continuous-time trajectory and the discrete-time trajectory at the discrete temporal nodes is analytically zero."* This requires a **single-shot nonlinear integration** per interval in addition to the STM propagation.

FOH control parameterization (Eq. 4): $u(\tau_k) = (1-\tau_k)u_k + \tau_k u_{k+1}$.

#### Single-crossing compound state-triggered constraints (Eq. 14a–14c)

$$g(x(t)) \ge 0 \quad \forall t\in[0,t_{trigger}) \tag{14a}$$
$$g(x(t)) = 0,\quad t = t_{trigger} \tag{14b}$$
$$\left.\begin{aligned} g(x(t)) &\le 0\\ c_1(x(t)) &\le 0\\ &\vdots\\ c_{n_c}(x(t)) &\le 0\end{aligned}\right\}\quad \forall t\in(t_{trigger},t_f) \tag{14c}$$

**This is the single most important methodological claim in file #2** (line 232): *"If the trigger conditions and the constraint conditions above are individually convex, the compound STC is entirely convex. This is in contrast to existing formulations of STCs in the literature that are inherently nonconvex."* The trigger must be **single-crossing**: $x^\star := g^{-1}(g^\star)$ is a singleton and $g(\cdot)$ is strictly monotonic in a neighborhood of $x^\star$. The example given is exactly the landing case: a rocket descending from altitude $h_i$ with target at the origin will *"certainly cross a trigger altitude $0 < h_{trigger} < h_i$ once during its descent, and not surpass that altitude after."*

#### Conic subproblem (Eq. 15a–15e) — strongly convex, matrix-inverse-free

$$\min_{u,s}\ w_cJ(x_N) + \tfrac{1}{2}\big(w_{tr}J_{tr} + w_{vse}J_{vse}\big) \tag{15a}$$
$$\text{s.t.}\quad x_{k+1} = \text{RHS of Eq. (11)},\ k=1{:}N-1 \tag{15b}$$
$$\xi_k \in \mathcal X,\ k=1{:}N \tag{15c}, \qquad u_k\in\mathcal U,\ k=1{:}N \tag{15d}, \qquad s_k\in\mathcal S,\ k=1{:}n_{phase} \tag{15e}$$

$$J_{tr} := \sum_{k=1}^{N}\Big(\|x_k - \bar x_k\|_2^2 + \|u_k - \bar u_k\|_2^2\Big) + \sum_{k=1}^{n_{phase}}\|s_k - \bar s_k\|_2^2, \qquad J_{vse} := \sum_{k=1}^{N}\|x_k - \xi_k\|_2^2$$

Vectorized form (Eq. 16a–16c): $\min_z \frac12 z^TQz + \langle q,z\rangle$ s.t. $Hz-h=0$, $z\in\mathcal D$.

#### Phase-specific constraints (the multi-phase payload)

- **Common** (Eq. 21–22): $s_{\min}\le s_l\le s_{\max}$, $l\in\{1{:}n_{phase}\}$; and $r_{x_k}^\xi \ge h_{trigger}$ for $k\in\{1{:}k_{trigger}-1\}$.
- **Coast** (Eq. 23): $m_1^\xi = m_i$, $r_1^\xi=r_i$, $v_1^\xi=v_i$, $\theta_1^\xi=\theta_i$, $\omega_1^\xi=\omega_i$, $T_1=0$. ZOH on control to avoid violation at ignition.
- **High-thrust (3-engine)** (Eq. 24–26): $3T_{\min}\le T_k\le 3T_{\max}$
  $$\max\{-\delta_{\max},\,-\dot\delta_{\max}\bar s_{k-1}+\bar\delta_{k-1}\} \le \delta_k \le \min\{\delta_{\max},\,\dot\delta_{\max}\bar s_{k-1}+\bar\delta_{k-1}\} \tag{25}$$
  $$\delta_{k_{switch}} = \bar\delta_{k_{switch}-1} \tag{26}$$
  > Eq. 25 is a **linear surrogate for the gimbal rate limit** obtained by replacing the true previous value with the *reference* value, making the bounds constants. The paper is candid (line 527): *"This constraint is, however, exact at convergence, and this formulation has been observed to work well in practice."* Same for the glideslope box form below.
- **Low-thrust (1-engine)** (Eq. 27–28): $T_{\min}\le T_k\le T_{\max}$; same gimbal form as Eq. 25.
- **Terminal descent** (Eq. 29–31): $T_{\min}\le T_k\le T_{\max}$;
  $$r^\xi_{x_k} \begin{cases} = h_{trigger}, & k=k_{trigger}\\ \le h_{trigger}, & \text{otherwise}\end{cases} \tag{30a}$$
  $$|r^\xi_{z_k}| \le \begin{cases} \tan\gamma_{gs}\,h_{trigger}, & k=k_{trigger}\\ \tan\gamma_{gs}\,\bar r^\xi_{x_k}, & \text{otherwise}\end{cases} \tag{30b}$$
  $$\qquad \|v^\xi_k\|_2\le v_{\max} \tag{30c},\quad |\theta^\xi_k|\le\theta_{\max} \tag{30d},\quad |\omega^\xi_k|\le\omega_{\max} \tag{30e}$$
  $$\max\{-\delta_{\max}^{TD},\,-\dot\delta_{\max}\bar s_{k-1}+\bar\delta_{k-1}\}\le\delta_k\le\min\{\delta_{\max}^{TD},\,\dot\delta_{\max}\bar s_{k-1}+\bar\delta_{k-1}\} \tag{30f}$$

  Terminal BCs (Eq. 31): $m^\xi_N\ge m_{dry}$, $r^\xi_N=r_f$, $v^\xi_N=v_f$, $\theta^\xi_N=\theta_f$, $\omega^\xi_N=\omega_f$, $\delta_N=0$.

  Objective: $J(x_N) = -m_N$ (maximize final mass ⇒ minimize propellant).

  **Note the convexification move on the glideslope** (line 562–564): *"the glideslope constraint is cast in the form of box constraints in terms of the reference values, in order to enable closed-form projections (without this measure, there would be two constraints on $r^\xi_{x_k}$ at every temporal node, thus precluding closed-form projections). Similar to the combined gimbal constraint, this constraint is exact at convergence."*

### 1.3 File #3 — Soldati TUM thesis 2025, 3-DoF fault-tolerant SCP

#### The natural convex/non-convex split (Eq. 6.4–6.9) — genuinely instructive

$$\dot x = f_{nc}(x) + f_c(x) + Bu \tag{6.4}$$

with the **convex part** $f_c(x) = \begin{bmatrix} v \\ 0_{3\times1} \\ -\frac{\sum_{i=1}^5 T_i}{I_{sp}g_0} \\ 0_{15\times1}\end{bmatrix} = A_cx$ and

$$A_c = \begin{bmatrix} 0_{3\times3} & I_{3\times3} & 0_{3\times16}\\ 0_{3\times3} & 0_{3\times3} & 0_{3\times16}\\ 0_{1\times7} & -\frac{1}{I_{sp}g_0}\mathbf{1}_{1\times5} & 0_{1\times10}\\ 0_{15\times3} & 0_{15\times3} & 0_{15\times16}\end{bmatrix} \tag{6.6}, \qquad B = \begin{bmatrix} 0_{7\times15}\\ I_{15\times15}\end{bmatrix} \tag{6.7}$$

$$f_{nc}(x) = \begin{bmatrix} 0_{3\times1}\\ a_{grav}+a_{prop}+a_{aero}\\ 0_{15\times1}\end{bmatrix} \tag{6.8}, \qquad \dot x = f(x,u) = A_cx + f_{nc}(x) + Bu \tag{6.9}$$

State is $22\times K$ = 7 physical (3 pos + 3 vel + 1 mass) + 15 augmented (5 thrusts + 8 TVC angles + 2 aero angles); control is $15\times K$ = the **time-derivatives** of those 15 augmented states (Eqs. 5.1–5.5). $A_c$ and $B$ are constant ⇒ *"they need to be computed only once"* (line 2908).

**Why the controls were moved into the state** (line 1736): *"To enforce explicit bounds on the control rates and to obtain smooth control sequences, the controls are reformulated such that the original control variables become states, while their time derivatives serve as the new control inputs."*

And a sharp numerical reason for keeping the control affine (line 2910, citing [46] = Liu/Shen/Lu 2016): *"there is no need to compute Jacobians with respect to the control variables. According to [46], linearization with respect to the control variables introduces high-frequency chatter in the control profile due to numerical instability, which is generally detrimental to the convergence of the successive solution process. Small oscillations in $u_{n-1}$ propagate into the elements of the linearized dynamics, and as a result the updated control $u_n$ is likely to amplify these chattering effects."*

#### Glideslope as a pure SOC (Eq. 5.7, 6.10–6.11)

$$\frac{x}{\sqrt{y^2+z^2}} \ge \tan\gamma_{gs} \tag{5.7}$$

$$\Big([y,z],\ x\tan\gamma_{gs}\Big) \in \mathcal K^3 \tag{6.10} \quad\Longleftrightarrow\quad \left\|\begin{bmatrix}y\\z\end{bmatrix}\right\|_2 \le x\tan\gamma_{gs} \tag{6.11}$$

The thesis notes (line 2922): *"The glideslope constraint (5.7) is conic by nature and therefore does not require convexification."* Fixed $\gamma_{gs}=45°$ in this work.

#### Free final time via $\sigma$ (Eq. 6.15–6.17)

$$\frac{dx(t)}{dt} = \frac{d\tau}{dt}\cdot\frac{dx}{d\tau} \tag{6.15}, \qquad \sigma := \left(\frac{d\tau}{dt}\right)^{-1} = \frac{dt}{d\tau} = t_f - t_0 = t_f \tag{6.16}, \qquad x'(\tau) = \sigma\cdot f(x(\tau),u(\tau)) \tag{6.17}$$

This is the **same $\sigma$ device as file #1** (and cites it as ref. [87] = Szmuk & Açıkmeşe 2018). $\sigma$ is an optimization variable.

#### Linearization + virtual control + trust region (Eqs. 6.18–6.36)

$$\dot x(\tau) = f(\tilde x(\tau),\tilde u(\tau))\sigma + A(\tau)x(\tau) + B_\sigma u(\tau) + z(\tau) \tag{6.18}$$
$$A(\tau) = \big(A_{nc}(\tau) + A_c\big)\tilde\sigma \tag{6.19}, \quad B_\sigma = B\tilde\sigma \tag{6.20}, \quad A_{nc}(\tau) = \left.\frac{\partial f_{nc}(x,u)}{\partial x}\right|_{\tilde x(\tau),\tilde u(\tau)} \tag{6.21}, \quad z(\tau) = -A(\tau)\tilde x(\tau) - B_\sigma(\tau)\tilde u(\tau) \tag{6.22}$$

Virtual control $\nu(\tau)\in\mathbb R^{22}$ (Eq. 6.23–6.27):
$$\dot x(\tau) = f(\tilde x,\tilde u)\sigma + A(\tau)x(\tau) + B_\sigma u(\tau) + z(\tau) + C\nu \tag{6.24}, \qquad C = I_{22} \tag{6.25}$$
$$\|\nu(\tau)\|_2 \le \kappa(\tau) \tag{6.26}, \qquad \|\kappa\|_2 \le s_\kappa \tag{6.27}$$

Trust region (Eq. 6.32–6.34):
$$\big\|X(\tau) - \tilde X(\tau)\big\| \le \eta(\tau) \tag{6.32}, \qquad \|\eta\|_2 \le s_\eta \tag{6.33}, \qquad \|\sigma - \tilde\sigma\| \le \eta_\sigma = \text{const} \tag{6.34}$$

with $X(\tau) = [x(\tau), u(\tau)]$. **Note:** states AND controls share one trust region, and $\sigma$ gets a **separate fixed-radius** trust region.

Integral bounded for a linear cost (Eq. 6.35):
$$\int_0^1 \ell_{convex}(\tau)\,d\tau \le s_R \tag{6.35}$$

Augmented cost (Eq. 6.36):
$$J = w_Rs_R + w_\eta s_\eta + w_\kappa s_\kappa + w_\sigma\sigma \tag{6.36}$$

$$\ell_{convex}(x,u) = \ell_c(x,u) + \ell_\tau(\tilde x,\tilde u) + \left.\frac{\partial\ell_\tau}{\partial x}\right|_{\tilde x,\tilde u}(x-\tilde x) + \left.\frac{\partial\ell_\tau}{\partial u}\right|_{\tilde x,\tilde u}(u-\tilde u) \tag{6.28}$$

Discretization is **trapezoidal** (Eq. 6.37–6.39), $\Delta\tau = \frac{1}{K-1}$, $\tau_k = \frac{k-1}{K-1}$:

$$x_{k+1} = x_k + \frac{\Delta\tau}{2}\Big[f_k^{n-1}\sigma + A_k^{n-1}x_k + B_\sigma u_k + z_k^{n-1} + C\nu_k + f_{k+1}^{n-1}\sigma + A_{k+1}^{n-1}x_{k+1} + B_\sigma u_{k+1} + z_{k+1}^{n-1} + C\nu_{k+1}\Big] \tag{6.39}$$

Glideslope enforced at each node (Eq. 6.45): $\big([y_k,z_k],\ x_k\tan\gamma_{gs}\big)\in\mathcal K^3,\ k=1,\dots,K$. Quadrature matches dynamics (Eq. 6.46), following Rao's survey.

**Slack chain, restated discretely** (Eqs. 6.52–6.55):
$$\|\nu_k\|_2\le\kappa_k \tag{6.52}, \qquad \|\kappa\|_2\le s_\kappa \tag{6.53}$$
$$\|x_k - x_k^{n-1}\|_2^2 + \|u_k - u_k^{n-1}\|_2^2 \le \eta_k \tag{6.54}, \qquad \|\eta\|_2\le s_\eta \tag{6.55}$$

#### Fault reconfiguration (Eqs. 6.58–6.67)

Final-horizontal-position relaxation:
$$|y(t_f)|\le s_y \tag{6.58}, \qquad |z(t_f)|\le s_z \tag{6.59}, \qquad J = w_{pos}(s_y+s_z) + w_\eta s_\eta + w_\kappa s_\kappa + w_\sigma\sigma + w_Rs_R \tag{6.60}$$

**Cone origin shifts with the achieved lateral position** (Eq. 6.61) — a clever feasibility-preserving trick:
$$\left\|\begin{bmatrix}y - y(t_f)\\ z - z(t_f)\end{bmatrix}\right\|_2 \le x\tan\gamma_{gs} \tag{6.61}$$

Total engine loss (F1): $T^i_{\min} = T^i_{\max} = 0$, $\dot T^i_{\min}=\dot T^i_{\max}=0$ (Eq. 6.62); TVC bounds and rates zeroed (Eqs. 6.63–6.64).
Thrust degradation (F2): $T^i_{\max} = \xi_i T^{i}_{\max,nom}$, $\frac{T_{\min,nom}}{T_{\max,nom}} < \xi_i < 1$ (Eq. 6.65).
TVC jamming (F3): $\beta^{\min}_{y,i} = \beta^{\max}_{y,i} = \beta^{jam}_{y,i}$, ditto $z$; rates zeroed (Eqs. 6.66–6.67).

### 1.4 The lossless-convexification $\sigma$ / log-mass device — NOT present in these files

The user asked specifically about "the lossless convexification variable change (thrust magnitude slack variable sigma, log-mass)". **None of the four files contains that formulation.** Verification:

- File #1 uses $\sigma$ for the **time-dilation coefficient** ($\sigma \triangleq (d\tau/dt)^{-1}$), which is an unrelated reuse of the same symbol. It mentions lossless convexification only in the Introduction (line 134) and cites refs. 13–20, including `Açıkmeşe, B. and Ploen, S. R., "Convex Programming Approach to Powered Descent Guidance for Mars Landing," AIAA JGCD, Vol. 30, No. 5, 2007, pp. 1353–1366` (ref. 17) and `Açıkmeşe, Carson, Blackmore, IEEE TCST Vol. 21, No. 6, 2013, pp. 2104–2113` (ref. 19).
- File #3 uses $\sigma$ for the **same time-dilation meaning** (Eq. 6.16).
- File #2 does not use $\sigma$ for either purpose; its dilation factor is $s_k$.
- File #4 mentions lossless convexification by name only, with no equations (§6 below).

**Actionable:** if the user wants the true $\sigma$/log-mass formulation, they must obtain Açıkmeşe & Ploen 2007 or Açıkmeşe/Carson/Blackmore 2013 directly — those citations are in file #1's reference list. What I can state from the *text present* is only that (file #1, line 134): *"Lossless convexification was one method proposed to circumvent the non-convex minimum thrust constraint of the landing problem. This method was successfully demonstrated numerous times during flight experiments"* (refs. 21–22 = G-FOLD/ADAPT).

---

## 2. Terminal attitude / tilt constraints in the 6-DoF formulations

### 2.1 The convex tilt constraint is ALREADY exact — no linearization needed (file #1)

This is a genuinely notable finding and it is stated explicitly. The tilt angle is defined by Eq. 8:

$$\cos\theta(t) = e_1\cdot C_{I/B}(t)e_1 = 1 - 2\big(q_2^2(t)+q_3^2(t)\big)$$

and the constraint $\theta(t)\le\theta_{\max}$ becomes, **by moving the quaternion terms to the right**:

$$\cos\theta_{\max} \le 1 - 2\big(q_2^2(t)+q_3^2(t)\big) \tag{9}$$

$$\text{compact form:}\qquad \cos\theta_{\max} \le 1 - 2\|H_q q_{B/I,k}^i\|_2^2 \tag{28a}, \qquad H_q \triangleq \begin{bmatrix}0&0&1&0\\0&0&0&1\end{bmatrix} \tag{28b}$$

The paper's claim (line 293): *"To avoid excessive tilt angles in the trajectory, we limit $\theta(t)$ to a maximum value of $\theta_{\max}$. If we immerse $q_{B/I}$ in $\mathbb R^4$, we can impose this limit through the following convex constraint."*

**Why it is convex without linearization:** the constraint set $\{q : 2(q_2^2+q_3^2) \le 1-\cos\theta_{\max}\}$ is the **exterior of a ball in the $(q_2,q_3)$ plane** — an upper bound on a convex quadratic. A sublevel set of a convex function is convex. The tilt limit bounds the *transverse* quaternion components, which is exactly a Euclidean-ball constraint on a 2-vector.

**This is class (a)/(b) usable — a direct analytic reduction.** On kOS, with no matrices needed:
```
// quaternion q = [q0,q1,q2,q3] (leading scalar), body->inertial
// tilt = angle between ship's up-ish body X axis and the inertial up axis
tiltCos = 1 - 2*(q2*q2 + q3*q3)      // == cos(theta)
// constraint: tiltCos >= cos(thetaMax)
```
In kOS the natural equivalent is `VDOT(facing:vector, up:vector)` vs. `cos(thetaMax)` — the algebraic form above is the exact convex-representation equivalent and is a **single dot product and two multiplies**.

Also note (line 312): the final thrust vector is constrained to point along the body X-axis, `e2·T_B(tf) = e3·T_B(tf) = 0`, *"in order to null out the torques at the final condition."* And the initial attitude is **free** while the final attitude is **fixed** to $q_{B/I,f}$; in the simulations $q_{B/I,f} = [1,0,0,0]^T$ (upright). The max angular rate is $\omega_{\max}$ via Eq. 10.

### 2.2 File #3 — attitude handled by an implicit inner loop, not by the optimizer

The TUM thesis deliberately **does not** put attitude in the guidance problem. From the modeling assumptions (lines 1719–1720):

> *"Only 3-DoF translational dynamics are modeled, attitude dynamics are represented implicitly through aerodynamic angles, whose values are assumed to be instantly tracked by a lower level controller."*

Instead, attitude is kept compatible with a 6-DoF realization by **cost shaping** rather than constraints (lines 1855–1856):

> *"The inclusion of the propulsive and aerodynamic torque penalties in the cost function is motivated by the fact that, despite a 3-DoF formulation, we want to ensure that the resulting solution remains compatible with a full six-degree-of-freedom (6-DoF) model. In other words, the computed trajectory must be trimmable."*

$$J_{\tau^{prop}} = \sum_{i=1}^{3}\tau_B^{prop}(i)^2 \tag{5.13}, \qquad J_{\tau^{aero}} = \sum_{i=1}^{3}\tau_B^{aero}(i)^2 \tag{5.14}$$

with weights $w_{\tau}^{prop}=10^{-2}$, $w_{\tau}^{aero}=10^{-4}$ (Table 5.1).

Terminal attitude is enforced as **boundary conditions on deflection/aero angles**, all driven to zero (Problem 1, lines 1989–1993):

$$\alpha(t_f)=0,\qquad \beta(t_f)=0,\qquad \beta_{y,i}(t_f)=0,\qquad \beta_{z,i}(t_f)=0,\qquad i=1,\dots,4$$

The vehicle lands with approximately 970 kg of fuel remaining, "corresponding to about 1% of the total propellant mass" (line 2051). TVC limits: $\beta^{\max}=8°$, rate $4°/s$ (Table 4.1). TVC box in the NLP: $\pm5°$ (Table 5.2).

### 2.3 File #2 — tilt as a simple box constraint in a 3-DoF model

Attitude is a **scalar** tilt angle $\theta$ in this planar model, so the constraint is a trivial linear box (Eq. 30d):

$$|\theta^\xi_k| \le \theta_{\max}, \qquad \theta_{\max} = 5°\ \text{(terminal phase)}, \qquad \omega_{\max}=2.5°\,\text{s}^{-1}$$

Initial tilt $\theta_i = 90°$ (belly-flop attitude), final $\theta_f = 0°$ (upright) (Eq. 31). The paper calls out the *purpose* of the tightly-constrained terminal phase (line 541):

> *"The final phase, the terminal descent phase, is the most heavily and tightly constrained phase of flight. This is designed as such in order to enable closed-loop precision landing, i.e., to ensure that the generated guidance trajectories (the feedforward control input signal and the reference state profiles) are amenable to tight tracking via feedback controllers. Such a phase would be especially useful if sub-meter touchdown accuracy is required."*

### 2.4 Summary table for question 2

| Formulation | Attitude representation | Terminal tilt handling | Convex? |
|---|---|---|---|
| File #1 (6-DoF) | Unit quaternion $q_{B/I}\in S^3$, $\omega_B$ | $\cos\theta_{\max}\le 1-2(q_2^2+q_3^2)$, exact; final $q$ fixed, $\omega=0$, final thrust along body X | **Yes, natively convex** |
| File #2 (3-DoF planar) | Scalar tilt $\theta$ | $\|\theta^\xi_k\|\le\theta_{\max}$ box; $\theta_f=0$, $\omega_f=0$ | **Yes, linear** |
| File #3 (3-DoF + aero) | *Not modeled*; implicit inner loop | Torque penalties in cost (§5.13–5.14); $\alpha,\beta,\beta_y,\beta_z \to 0$ at $t_f$ | **Yes, cost only** |

**No file contains an example of a *linearized* tilt constraint** (e.g. a Taylor expansion of a tilt-angle inequality). File #1's contribution is precisely that the natural quaternion form is already convex. That is a genuinely useful negative result for the user: **do not linearize the tilt constraint — use the quadratic form of Eq. 9.**

---

## 3. Bang-bang thrust structure, switches, min/max arcs, max-then-min

### 3.1 File #1 — explicit, direct statements

This is the single most useful extraction for the user's problem, because it justifies an extremely simple control structure.

**Line 842 (2-D in-plane example), verbatim:**
> *"The trajectory is seen to ride the maximum tilt angle limit during the first half of the trajectory, and the angular velocity limit during most of the second half. Throughout, **both the minimum and maximum thrust magnitude bounds are activated, as one would expect for a minimum-time trajectory.**"*

**Line 846 (3-D out-of-plane example), verbatim:**
> *"Despite the more complex nature of the trajectory, the algorithm performed nearly identically when compared to the 2-D case. Here, **bang-coast-bang features** and angular rate saturation similar to those observed in the 2-D case are apparent in the thrust magnitude and angular rate profiles, respectively."*

**Quantitative structure observed:**
- The thrust magnitude profile oscillates between the bounds $T_{\min}=0.30$ and $T_{\max}=5.00$ (nondimensional, Table 1).
- The pattern is **bang–coast–bang**: a max-thrust arc, then a lower/coast arc, then a max-thrust arc.
- This is **minimum-time**, and the paper says this is *expected* — "as one would expect for a minimum-time trajectory."

**Important precision on "max-then-min".** The paper says **bang-coast-bang** and **both bounds activated**, and that the profile is a *minimum-time* profile. It does **not** say "max-thrust-then-min-thrust" as a strict two-arc statement, and it does **not** report a switch count. The literal reading supported by the text is:
- ≥2 saturated max-thrust arcs, separated by a coast/lower-thrust arc;
- min-thrust bound also activated somewhere;
- bang-coast-bang is the observed signature.

**Caveat to state plainly:** this is a **minimum-TIME** problem. The user's kOS scenario is implicitly a **fuel/feasibility** problem. Minimum-fuel solutions have a **different** structure in general (typically max-thrust to a singular arc, then a min-thrust/singular terminal arc). The file provides no minimum-fuel structural theorem — it explicitly notes its 6-DoF fixed-final-time minimum-fuel work is separate prior work (ref. 2). **Do not transfer the bang-coast-bang conclusion to a min-fuel objective without independent justification.**

### 3.2 File #3 — observed bang structure is *relative to a nominal reference*

From the SCP-vs-GPOPS comparison (line 3414):
> *"Small discrepancies appear in the control profiles: most notably, **a lower thrust rate during the first and only thrust bang**, which results from the higher penalization weight for the thrust derivative in SCP."*

So the nominal minimum-… (here, weighted-integral) solution exhibits **"the first and only thrust bang"** — a single bang. The mission is a 5 km / ~60 s PDL from an initial state $[5000, 100, -200]$ m, $[-219.43, -13.17, -8.78]$ m/s (Table 5.2). Thrust bounds: $T_{\max}=58$ kN, $T_{\min}=17.4$ kN, $\dot T_{\max}=27.1$ kN/s (Table 4.1) — i.e. $T_{\max}/T_{\min} = 3.33$.

Under a total engine loss (F1, one of five engines out at $t_{fault}=15$ s), line 4114:
> *"To compensate for the lost engine, the remaining healthy engines **ignite approximately 10 s earlier** than in the nominal case, **reach maximum thrust, and sustain it for a longer duration** ... which results in an additional propellant consumption of about 200 kg."*

Under thrust degradation (F2, $\xi_1=60\%$ at $t_{fault}=25$ s), line 4402:
> *"The healthy engines respond by **transitioning to the maximum thrust immediately at the fault time**, while the degraded engine approximately 10 s later, reach maximum thrust, and sustain it for a longer duration ... which results in an additional propellant consumption of about 30 kg."*

**Implementation deduction (marked as deduction, not quotation):** the pattern is consistently *"saturate at $T_{\max}$ as early and as long as the remaining budget allows"*, with a lower-thrust final trimming arc. The fault responses are all "burn harder, earlier."

F3 (TVC jamming) is the instructive counterexample (line 4659): the faulty engine *"never reaches or maintains the maximum value and gradually decreases, returning to the minimum at touchdown"* — i.e. when a vehicle constraint (not a time/fuel constraint) binds, the thrust structure changes qualitatively.

### 3.3 File #2 — the optimizer times the engine switches itself

SeCO's selling point is that free-switch-time multi-phase optimization needs **no mixed-integer constraints** (line 93):
> *"By treating a phase-based subset of $s_k$ ... as decision variables and discretizing the system over them, we allow the optimizer to decide what the temporal spacing of discrete nodes should be in each phase rather than use a uniform temporal grid over the entire horizon. In the approach we propose, this is the key to enabling free-transition-time multi-phase trajectory optimization within a single-shot optimization framework, without requiring any mixed-integer or nonconvex constraints to handle the discrete temporal events/switching."*

The result (line 605):
> *"the optimizer chooses to **initiate the powered-descent phase at an altitude of 490.34 m and a speed of 86.28 m s⁻¹** (which is very close to the terminal velocity)."*

with $v_{terminal}=85$ m s⁻¹ given as a parameter. So the optimizer **waits until near terminal velocity before igniting** — a strong, transferable heuristic.

Multi-engine thrust bounds change discretely at the switch: $3T_{\min}\le T\le 3T_{\max}$ → $T_{\min}\le T\le T_{\max}$ with $T_{\max}=2200$ kN, $T_{\min}=880$ kN (ratio 2.5).

### 3.4 Verdict for question 3

| Claim | Where | Usable on kOS? |
|---|---|---|
| Min-TIME ⇒ both thrust bounds activated; bang-coast-bang | File #1, lines 842, 846 | **(b)** — justifies a 2–3 arc saturation law |
| "First and only thrust bang" in the nominal min-integral solution | File #3, line 3414 | **(b)** — supports a one-switch max-then-trim law |
| Fault response = saturate $T_{\max}$ earlier and longer | File #3, lines 4114, 4402 | **(b)** — direct rule for a degraded booster |
| Ignite only near terminal velocity | File #2, line 605 | **(b)** — usable ignition-timing heuristic |
| A **proof** of max-then-min optimality | *nowhere in these four files* | — |

---

## 4. Free final time handling — how $t_f$ is found

Four distinct mechanisms appear. The user asked about line search, sigma variable, and transversality; all three appear, plus a fourth.

### 4.1 File #1 — NO line search; $\sigma$ is a decision variable

**Explicit statement (line 143), verbatim:**
> *"We emphasize that **our method does not perform a line search on the time-of-flight.** Furthermore, in contrast to existing heuristic strategies, our method is capable of generating trajectories that are dynamically feasible, that adhere to the prescribed constraints, and that are more optimal, thus enlarging the usable flight envelope."*

Mechanism: normalize time to $\tau\in[0,1]$, introduce $\sigma \triangleq (d\tau/dt)^{-1}$ (Eq. 15), get $x'(\tau) = \sigma f(x(\tau),u(\tau))$ (Eq. 16). Because $\tau_f = 1$ always, the problem becomes **fixed-final-time** in $\tau$ — the paper says exactly this (line 615): *"By virtue of the time normalization introduced in Section III.A.1, this problem can be viewed as a fixed-final-time optimization problem due to the fact that the final normalized time is always equal to unity. As a consequence, $t_f$ in Problem 1 is substituted with $\sigma^i$."*

$\sigma$ enters the linearized dynamics through $\Sigma(\tau) \triangleq f(\hat x(\tau),\hat u(\tau))$ (Eq. 17d) and the discrete $\bar\Sigma_k$ (Eq. 22e). It is bounded only through the trust region $\delta\sigma^i\cdot\delta\sigma^i \le \Delta_\sigma^i$ (Eq. 24b) with penalty $w_{\Delta\sigma}\|\Delta_\sigma^i\|_1$ (Eq. 25).

**Convergence history (Fig. 2 description and text, lines 843–846):**
> *"the algorithm was initialized using **ten different time-of-flight guesses, ranging from 1.0 [UT] to 10.0 [UT], in 1.0 [UT] increments. All ten initializations generated the same converged trajectory, yielding time-of-flights within 0.01 [UT] of each other.**"*

Convergence was reached by **iteration 6** in the 2-D case and **iteration 9** in the 3-D case, despite a **10× spread** in the initial $t_f$ guess. **This is a strong robustness result: the initial $t_f$ guess essentially does not matter.**

### 4.2 File #2 — time-interval dilation: $s_k$ is the interval LENGTH

$$\tau_k(t) := \frac{t-t_k}{t_{k+1}^- - t_k},\qquad s_k := t_{k+1}^- - t_k \in\mathbb R_+ \tag{2–3}$$

Constrained by $s_{\min}\le s_l\le s_{\max}$ (Eq. 21). With $N=16$ nodes and $s_{\min}=0.6$ s, $s_{\max}=10$ s, the effective horizon is $[0, N-1)$ after dilation (line 95).

Partitioning choice and rationale (line 94): *"Although it is possible to allow each dilation factor to be an independent decision variable, we choose to partition the temporal grid based on the phases of flight, and evenly space the temporal nodes within each phase. This measure is taken to mitigate extreme inter-sample constraint violation, which tends to occur when fully adaptive grids are used."*

**This is the most kOS-translatable $t_f$ mechanism in all four files:** ~4 scalar decision variables (one per phase) instead of a per-node grid.

### 4.3 File #3 — $\sigma$ as an optimization variable, PLUS an explicit small penalty $w_\sigma\sigma$

$$\sigma := \left(\frac{d\tau}{dt}\right)^{-1} = \frac{dt}{d\tau} = t_f - t_0 = t_f \tag{6.16}, \qquad x'(\tau) = \sigma\cdot f(x(\tau),u(\tau)) \tag{6.17}$$

Bounded by a **fixed-radius** trust region: $\|\sigma - \tilde\sigma\|\le\eta_\sigma = \text{const}$, with $\eta_\sigma = 0.5$ (Eq. 6.34, Table 6.2).

Cost includes $w_\sigma\sigma$ with $w_\sigma = 10^{-2}$ (Eq. 6.36). The rationale (line 3416) is a genuinely useful practical note:
> *"a small penalty on the final time is introduced, as **numerical experimentation showed that this improves the agreement with the GPOPS-II solution.**"*

**Convergence of $\sigma$ (line 3417), verbatim and quantitative:**
> *"Starting from the initialization at $\sigma = 1$, the time-dilation coefficient increases to its realistic value of **about 6** within the **first eight iterations**. During this phase, the virtual-control slack variable remains non-negligible, since the discretized dynamics constraint in (6.39) cannot be satisfied without artificial inputs. Once $\sigma$ stabilizes around 6, the magnitude of the virtual controls rapidly vanishes to essentially zero (numerical values on the order of $10^{-18}$, i.e. machine precision). From that point onward, both the integral slack and the trust region slack decrease, leading to **convergence at iteration 13**."*

Note the scaling: normalized $\sigma=6$ with time scale $L/V = 5000/500 = 10$ s ⇒ $t_f \approx 60$ s, consistent with the plots (0–60 s).

### 4.4 Transversality — file #3, and it is the one directly usable on kOS

**Eq. 5.16–5.18, verbatim:**

$$H = \lambda_r^Tv + \lambda_v^Tf_v(x,u) - \lambda_m\frac{\sum_{i=1}^5 T_i}{I_{sp}g_0} + \sum_{i=1}^5\lambda_{T,i}\dot T_i + \sum_{i=1}^4\lambda_{\beta_y,i}\dot\beta_{y,i} + \sum_{i=1}^4\lambda_{\beta_z,i}\dot\beta_{z,i} + \lambda_\alpha\dot\alpha + \lambda_\beta\dot\beta + \ell(x,u) \tag{5.16}$$

> *"Since the Hamiltonian does not depend explicitly on time, it remains constant along the optimal trajectory. Moreover, as this is a free final time problem, the transversality condition [21] requires"*

$$H(t_f) = 0 \tag{5.17}$$

> *"Hence, the Hamiltonian along the optimal trajectory satisfies"*

$$H(t) = 0,\qquad t\in[t_0,t_f] \tag{5.18}$$

Verified numerically (line 2358): *"Figure 5.3 shows the reconstructed Hamiltonian, which remains close to zero throughout the trajectory, thereby confirming the optimality of the solution."* (The plot y-axis is $\pm 6\times10^{-3}$, so residuals are $O(10^{-3})$.)

**Why this matters for the user:** $H \equiv 0$ is a **scalar, cheap, offline or slow-rate consistency check** on a candidate $t_f$. It does not require a solver — it requires costates, which the user does not have. Marked **(c)** for on-board use, **(b)** if the user is willing to accept an approximate $H$ built from a linear-quadratic *estimate* of the costates. See §7 for the practical substitute.

### 4.5 Summary for question 4

| Mechanism | File | $\sigma$ role | Bounds on $t_f$ | Reported convergence |
|---|---|---|---|---|
| $\sigma$ as decision var, trust-region bounded | #1 | time dilation | $\delta\sigma\cdot\delta\sigma\le\Delta_\sigma$ | iter 6 (2-D), iter 9 (3-D); 10/10 initial guesses 1–10 UT → same $t_f$ within 0.01 UT |
| Time-interval dilation $s_k$ | #2 | interval length | $s_{\min}=0.6$ s, $s_{\max}=10$ s | 7 seco iterations |
| $\sigma$ + fixed-radius trust region + $w_\sigma\sigma$ penalty | #3 | time dilation | $\|\sigma-\tilde\sigma\|\le0.5$ | $\sigma$: 1→6 in 8 iters; full convergence iter 13 |
| Transversality $H(t_f)=0$ | #3 | — | — | verified a posteriori, residual $O(10^{-3})$ |

**Explicit negative result:** file #1 states it does **not** use a line search. **No file in this set recommends or describes a line search on $t_f$** as a good practice — file #1 frames it as the inferior "existing heuristic strategy."

---

## 5. Convergence / robustness tricks — radii, weights, initialization, iterations, timings, warm start

### 5.1 Trust region radius adaptation

**File #1** — the radius **is a decision variable**, bounded and penalized:
$$\delta x_k^i\cdot\delta x_k^i + \delta u_k^i\cdot\delta u_k^i \le e_k\cdot\bar\Delta^i \tag{24a}, \qquad \delta\sigma^i\cdot\delta\sigma^i \le \Delta_\sigma^i \tag{24b}$$
$$c_\Delta^i \triangleq w_\Delta^i\|\bar\Delta^i\|_2 + w_{\Delta\sigma}\|\Delta_\sigma^i\|_1 \tag{25}$$
with $w_\Delta^i = 10^{-3}$, $w_{\Delta\sigma} = 10^{-1}$ (Table 2). Trust regions centered at $x_k^{i-1}$, $u_k^{i-1}$, $\sigma^{i-1}$.

**File #3** — three-way discussion of options (lines 3084–3085), then a choice:
> *"One option is to prescribe a constant, user-specified trust region radius [44]. This approach decreases the optimization subproblem size since the radius is treated as fixed input rather than an optimization variable. Another strategy is to define **update rules** for the trust region radius, **expanding or contracting it according to measures of how well the linear model captures the nonlinear dynamics** [50, 92]. This approach comes with the benefit of still keeping the optimization problem size contained ... while taking into account other information and updating its value between iterations. A further alternative is to embed the trust region bounds directly into the optimization problem itself [67, 88]. **Adopting the latter idea**, we constrain the change between consecutive solutions by augmenting the problem with a slack variable that limits the maximum allowable deviation."*

So file #3 uses the **embed-as-slack** variant (Eqs. 6.32–6.34), and **does not** implement adaptive radius rules. It says so plainly. **Consequence: no concrete adaptive-radius update rule is given in any of the four files.** Both adaptive strategies are cited ([50] = Mao/Szmuk/Açıkmeşe 2018; [92] = Wang & Lu 2020) but not reproduced.

**File #2** — soft trust region via the PTR (penalized trust region) algorithm:
$$J_{tr} := \sum_{k=1}^{N}\Big(\|x_k-\bar x_k\|_2^2 + \|u_k-\bar u_k\|_2^2\Big) + \sum_{k=1}^{n_{phase}}\|s_k-\bar s_k\|_2^2$$
with weight $w_{tr}$. Problem (15) is stated to be **strongly convex**.

### 5.2 Virtual control / artificial infeasibility penalties

| File | Mechanism | Penalty weight | Convergence test |
|---|---|---|---|
| #1 | Virtual control $\nu_k^i\in\mathbb R^{14}$ added to dynamics (Eq. 26); $\bar\nu^i\in\mathbb R^{14(K-1)}$; cost term $c_\nu^i \triangleq w_\nu\|\bar\nu^i\|_1$ (Eq. 27) | $w_\nu = 10^5$ | $\|\bar\nu^i\|_1 \le \nu_{tol}$, $\nu_{tol}=10^{-10}$ |
| #3 | $\nu(\tau)\in\mathbb R^{22}$, $C=I_{22}$ (Eqs. 6.23–6.25); bounded by $\kappa$; $\|\nu\|\le\kappa$, $\|\kappa\|\le s_\kappa$; cost $w_\kappa s_\kappa$ | $w_\nu = 10^5$ | $s_\kappa\le\epsilon_\nu$, $\epsilon_\nu = 10^{-9}$ |
| #2 | **Virtual STATE** $\xi$ (not virtual control) — a copy of the state; dynamics on $x$, state constraints on $\xi$; $J_{vse} = \sum\|x_k-\xi_k\|_2^2$ | $w_{vse}$ | implicit via penalty |

**File #2's virtual-state rationale (lines 240–242) is a real conceptual advance worth quoting:**
> *"We propose a new approach to handling artificial infeasibility by means of a virtual state variable, which serves as a copy of the original state. This approach helps decouple the dynamics and control constraints from the state constraints and exactly satisfy all the path constraints at each solver iteration, while ensuring that the subproblem never turns infeasible. If $x$ is the actual state variable, $u$ is the control variable, and $\xi$ is the virtual state variable, the dynamics constraint is imposed on $x$ and $u$, the control constraints are imposed on $u$, and **all the state constraints are imposed on $\xi$**. To ensure that the dynamics and all other constraints are satisfied at convergence, we minimize the error between $x$ and $\xi$ by heavily penalizing the squared distance between them in the objective function. **The virtual state does not alter the dynamics manifold (unlike the virtual control approach), and preserves the shapes of the conic state constraint sets (unlike the virtual buffer approach).**"*

with a convexity proof (Proposition 1):
> *"Let $y_k := (x_k,\xi_k)$. $\therefore \|x_k-\xi_k\|_2^2 = y_k^TMy_k$, where $M = \begin{pmatrix}1&-1\\-1&1\end{pmatrix}\otimes I_{n_x}$. Since spec $M\in\{0,2\}$, $M$ is positive semidefinite (PSD)."*

**File #3 adds a two-level slack hierarchy** that is a nice numerical trick: each $\nu_k$ is bounded by its own $\kappa_k$, and the whole vector $\kappa$ is bounded by a single scalar $s_\kappa$ — so one scalar in the cost controls the global virtual-control budget (Eqs. 6.52–6.53). Same for $\eta$/$s_\eta$.

### 5.3 Initialization strategies (four distinct, all "cheap and dumb")

**File #1, Algorithm 1 — linear interpolation in normalized time:**
$$\alpha_1 \triangleq \frac{K-k}{K},\qquad \alpha_2 \triangleq \frac{k}{K}$$
$$m_k^0 = \alpha_1m_{wet}+\alpha_2m_{dry},\quad r_{I,k}^0 = \alpha_1r_{I,i},\quad v_{I,k}^0 = \alpha_1v_{I,i}+\alpha_2v_{I,f}$$
$$q_{B/I,k}^0 = [1,0,0,0]^T,\quad \omega_{B,k}^0 = 0,\quad u_k^0 = -m_k^0g_I,\quad \sigma^0 = t_{f,guess}$$

Note $u_k^0 = -m_k^0g_I$: initialize the thrust vector to exactly cancel gravity — a **hover-thrust initial guess**. And $\sigma^0 = t_{f,guess}$, with the paper's claim (line 65) that the method *"can be initialized with a simple, dynamically inconsistent reference trajectory"* and (line 140) *"First, we initialize the process using a simple, dynamically inconsistent reference trajectory."*

**File #3, §6.2.4:** *"The initial guess for the position and velocity states is obtained by linearly interpolating between the initial and final boundary conditions. The thrust magnitudes are initialized and held constant at their minimum value, while the TVC and aerodynamic angles are set to zero. All control rates are also initialized to zero."* Plus $x_0$ from the actual current state and $\sigma^0=1$.

**File #3, §6.3.4 — the fault-recovery re-initialization (very relevant to a re-planning booster):**
> *"In order to accelerate convergence, the standard line-initialization approach is replaced with a scheme that exploits the nominal trajectory. Specifically, **the algorithm is initialized using slices of the nominal state and control trajectories from $t_{fault}$ to the final time, modified to reflect the specific fault condition. The final time guess is also initialized with the corresponding nominal value.**"*

Also (line 4108): the fault-time formulation *"enforces not only the state of the rocket at the time of the fault but also **constrains the augmented state variables to coincide with the commands being applied by the controller. This prevents the guidance solution from returning control profiles that differ significantly from the current commanded values.**"* — a continuity/jerk guard worth copying.

**File #2:** initial conditions imposed on the virtual state (Eq. 23) with $T_1 = 0$ at the first node.

### 5.4 Reported iterations and solve times — the complete set

| Source | Iterations | Solve time | Notes |
|---|---|---|---|
| File #1, 2-D | **converged by iteration 6** | **none reported** | Max allowed $N_{iter,\max}=15$; $K=50$ discretization points; 10/10 $t_f$ guesses converged |
| File #1, 3-D | **converged after 9 iterations** | **none reported** | Same $K=50$, $N_{iter,\max}=15$ |
| File #2 | **7 seco iterations** | **pipg mean 13.7 ms**; ECOS mean **37.1 ms** (2.7× faster) | Averaged over **100 full seco solves**; propellant difference between solvers **0.02%** |
| File #3, nominal SCP | **convergence at iteration 13** | **none reported** | $N_{\max}=30$; $\sigma$: 1→6 over 8 iterations |
| File #3, FT-SCP | not reported per-run | **explicitly NOT real-time** | See below |

**File #1 is explicit that timings were not measured** (line 1094):
> *"In future work, we plan to explore the convergence properties of the successive convexification technique outlined in this paper, **present real-time timing statistics**, and incorporate simple aerodynamic effects into the 6-DoF problem."*

Simulation environment for file #1 (line 752): *"generated in MATLAB using CVX and SDPT3"*, with notional non-dimensional quantities — *"the simulation results were generated using notional non-dimensional quantities, and thus are not intended to match a real-world system."* **No absolute times, no absolute distances.** All parameters (Table 1): $g_I=-e_1$ [UL/UT²], $m_{wet}=2.00$, $m_{dry}=1.00$, $T_{\min}=0.30$, $T_{\max}=5.00$, $\delta_{\max}=20°$, $\theta_{\max}=90°$, $\gamma_{gs}=20°$, $\omega_{\max}=60°$/UT, $J_B=10^{-2}I_{3\times3}$, $r_{T,B}=-10^{-2}e_1$; $r_{I,i}=[4,4,0]^T$ [UL], $v_{I,f}=-0.1e_1$ [UL/UT], $\omega_{B,i}=0$, $q_{B/I,f}=[1,0,0,0]^T$; $w_\nu=10^5$, $w_\Delta^i=10^{-3}$, $w_{\Delta\sigma}=10^{-1}$, $\nu_{tol}=10^{-10}$, $\Delta_{tol}=10^{-3}$.

**File #3's candid non-real-time admission (lines 5612–5613), a crucial expectation-setter:**
> *"Third, although convex optimization was selected for its predictable runtime and convergence guarantees, **the current implementation is not yet online-capable. Convergence times remain too long for real-time applications**, primarily due to (i) numerical computation of Jacobians via finite differences, and (ii) reliance on CVX as an intermediate modeling layer. Both aspects introduce significant computational overhead."*

Implementation specifics (line 4916–4922): Simulink closed loop; FT-SCP as a MATLAB system block in a **triggered subsystem** triggered at $t_{fault}$; guidance/control at **50 Hz**; trajectories stored in **time-scheduled lookup tables**; *"The scheduling parameter is reset to zero at $t_{fault}$ and synchronized with the time vector produced by FT-SCP."*

### 5.5 Convergence criteria (exit conditions)

**File #1, Algorithm 1 step 3:**
$$\text{if } \big(\|\bar\Delta^i\|_2 \le \Delta_{tol}\big)\ \text{and}\ \big(\|\bar\nu^i\|_1 \le \nu_{tol}\big)\ \text{exit}$$
with $\Delta_{tol}=10^{-3}$, $\nu_{tol}=10^{-10}$.

**File #3, Eqs. 6.56–6.57:**
$$s_\kappa \le \epsilon_\nu \tag{6.56}, \qquad s_\eta \le \epsilon_\eta \tag{6.57}$$
with $\epsilon_\nu = 10^{-9}$, $\epsilon_\eta = 10^{-2}$ (Table 6.2).

**File #3's subtle failure-mode diagnosis (line 5099) — a genuinely useful warning:**
> *"It is important to note that the orange (non-converged) dots with values well below 0, similar to those of the blue converged cases, can be misleading. This does not mean that the algorithm failed to converge despite a feasible solution. One must recall that in order to converge, the algorithm also includes the condition on the trust-region difference between subsequent iterations (6.57). **If this difference is not sufficiently small, the algorithm is considered to have not converged.** This is precisely the case here: although a trajectory with negligible virtual controls is found, it is obtained through a large deviation in states and controls from the previous iteration rather than through a small step within the linearized region. For this reason, the solution is still considered infeasible and therefore the algorithm does not reach convergence."*

### 5.6 Warm starting

**File #2 is the only file that mentions warm starting explicitly** (line 324):
> *"We leverage the **warm-starting capability of pipg** and use the **$\ell_2$-hypersphere preconditioning** technique described in (Kamath et al., 2023) to further accelerate convergence."*

The PIPG iteration itself (Eq. 17a–17d):
$$z^{j+1} = \pi_{\mathcal D}\big[\zeta^j - \alpha\big(Q\zeta^j + q + H^T\eta^j\big)\big] \tag{17a}$$
$$w^{j+1} = \eta^j + \beta\big(H\big(2z^{j+1}-\zeta^j\big) - h\big) \tag{17b}$$
$$\zeta^{j+1} = (1-\rho)\zeta^j + \rho z^{j+1} \tag{17c}, \qquad \eta^{j+1} = (1-\rho)\eta^j + \rho w^{j+1} \tag{17d}$$
with step sizes $\alpha,\beta$ per (Yu et al. 2022a, Lemma 2) and extrapolation factor $\rho\in[1,2)$.

**File #1 and #3 do not mention warm starting.** File #3's re-initialization-from-nominal (§6.3.4) is the closest analogue and is effectively a warm start in trajectory space.

**Critical algorithmic distinction for the user:** PIPG is **first-order and matrix-inverse-free** (line 51: *"entirely devoid of matrix factorizations and inversions"*). This is exactly the property a resource-starved platform wants — but it still needs the projection operator $\pi_{\mathcal D}$ and the LTV matrices $A_k,B_k^\pm,S_k$, so it is still far out of kOS reach. It is **(c)** for this project, but conceptually the most "kOS-shaped" solver of the three.

### 5.7 Other robustness techniques worth noting

- **Scaling (file #3, Table 5.3).** *"A standard approach is to apply a transformation of units so that optimization variables lie approximately within the range $[-1,1]$. Four fundamental scaling quantities are typically selected: length, speed, time, and mass."* Values: $L=5000$ m, $V=500$ m/s, $T=L/V=10$ s, $M=4300$ kg, acceleration $V/(L/V)=50$ m/s², thrust $MV/(L/V)=2.1\times10^5$ N. **Directly applicable insight for kOS: work in normalized units.**
- **Weight sensitivity (file #3, lines 5609–5611), a stark warning:** *"the algorithm is **highly sensitive to the numerical values chosen for the cost function weights. Small variations in these values can drastically alter the solutions, and in some cases represent the difference between convergence and divergence.** This sensitivity complicates the tuning process and raises questions about robustness."*
- **A weight-tradeoff data point:** for one infeasible F3 case, *"decreasing this weight from its original value of **50 to 1** leads to [convergence] ... with lateral touchdown positions of 100 m and 150 m in the y and z directions, respectively."*
- **Feasibility verification via independent integration.** Both #1 and #3 validate by re-propagating: file #3 uses *"a Runge–Kutta 45 propagation (ode45 ... Dormand–Prince)"* of the initial conditions with interpolated controls, confirming *"errors on the order of 2 m in position"* (GPOPS baseline) and *"on the order of 3 m in position"* (SCP). File #1 achieves it structurally via the LTV matrices.
- **Chatter avoidance** (file #3, line 2910): never linearize w.r.t. controls — keep the system control-affine. This is a concrete, transferable numerical hygiene rule.

---

## 6. E-guidance / Apollo polynomial guidance vs convex approaches

I searched all four files for `E-guidance`, `Apollo`, `polynomial`, `analytical`, `Klumpp`, and `sub-optimal`. **The total yield across all four files is three passages, none of which is a comparison.**

### 6.1 File #1 — the only substantive statement

**Line 132, verbatim (this is the complete passage):**
> *"Several insightful methods have been proposed$^{4-6}$ to analytically obtain sub-optimal guidance trajectory solutions. Although some methods successfully incorporated control constraints, attitude, and attitude-rate into the problem,$^7$ these **analytical solutions could only handle a limited set of typical mission constraints.**"*

The cited references are:
- **ref. 4**: `Klumpp, A. R., "Apollo Lunar Descent Guidance," Automatica, Vol. 10, 1974, pp. 133–146.` — i.e. the Apollo / E-guidance lineage.
- **ref. 5**: `Najson, F. and Mease, K. D., "Computationally Inexpensive Guidance Algorithm for Fuel-Efficient Terminal Descent," JGCD, Vol. 29, No. 4, 2006, pp. 955–964.`
- **ref. 6**: `Topcu, U., Casoliva, J., and Mease, K. D., "Minimum-Fuel Powered Descent for Mars Pinpoint Landing," J. Spacecraft and Rockets, Vol. 44, No. 2, 2007, pp. 324–331.`
- **ref. 7**: `Sostaric, R. R. and Rea, J. R., "Powered descent guidance methods for the moon and mars," AIAA GNC, San Francisco, 2005.`

**What is actually claimed:** analytic methods give **sub-optimal** solutions and **can only handle a limited set of typical mission constraints**. That is the entire critique.

**What is NOT claimed — I checked and it is absent:** no statement that E-guidance fails to meet terminal position/velocity constraints; no accuracy comparison; no mention of the two-parameter (thrust direction / thrust magnitude) structure; no numerical comparison of any kind. The user's own note-file title `2025_分岔法动力下降制导律_指出E制导仅能满足初末位置速度约束.md` (a *different* file, not in my assignment) evidently carries that claim — **but it is not in any of the four files I was given, and I will not attribute it to them.**

### 6.2 File #1 — the positive case for convex, verbatim (line 133)

> *"In contrast, convex optimization has been considered a prime candidate for on-board autonomous guidance applications due to its **deterministic behavior, its global optimality guarantees, its ability to provide certificates of convergence and infeasibility, and the availability of efficient interior point method (IPM) algorithms to solve convex problems.**"*

And the stated motivation for the whole paper (line 143):
> *"in contrast to existing heuristic strategies, our method is capable of generating trajectories that are dynamically feasible, that adhere to the prescribed constraints, and that are more optimal, **thus enlarging the usable flight envelope.**"*

### 6.3 Files #2 and #3 — no E-guidance comparison

**File #2** mentions pseudospectral methods only, and critiques *those* (line 63): *"Despite their ability to solve multi-phase problems, pseudospectral methods are typically untenable for real-time implementations."* It positions lossless convexification historically (line 55): *"One such method, known as lossless convexification, was **the first convex optimization-based algorithm to compute a rocket landing guidance trajectory for a mid-flight large divert maneuver onboard the vehicle.**"*

**File #3** discusses offline-vs-online, not analytic-vs-convex. Its relevant framing (line 798):
> *"general NLP problems provide **no guarantees of finding a feasible solution, may converge only to local minima, and are often sensitive to initial guesses**. As a result, they cannot reliably ensure safe guidance in real time. Furthermore, **unpredictable computational time and the absence of assured algorithm convergence exclude the use of such methods for real-time applications.**"*

and on convexity's appeal with its limits (line 805, 817):
> *"convex problems can be solved to global optimality under mild assumptions, with guaranteed convergence in polynomial time."*
> *"LCvx formulations for more realistic lander models, such as full six-degree-of-freedom (6-DoF) dynamics or cases including complex aerodynamic forces, **remain underdeveloped, with only simplified examples studied to date.**"*

**File #3's LCvx vs SCP taxonomy (lines 814–819), useful context:**
1. **LCvx** — *"reformulates certain classes of nonconvex (OCPs) into higher-dimensional convex problems. Crucially, it can be shown that the solution of the convex problem is also an optimal solution to the original nonconvex problem, without excluding any feasible solutions. This 'lossless' property ensures that if a feasible solution exists, the convexified problem will find it."*
2. **SCP** — *"When LCvx cannot be applied... iteratively solve a sequence of convex subproblems obtained by linearizing the nonconvexities around the solution at the previous iteration."* Named variants: **SCvx** [50] and **GuSTO** [19].

Thesis's own choice rationale (line 856): *"A 3-DoF translational model with a cluster of engines was selected. The multi-engine setup is essential to allow for fault-tolerance, as one engine would not be enough, while the 3-DoF formulation was preferred over a full 6-DoF model for two reasons: **(i) real-time applicability requires reduced computational burden, and (ii) the simplified translational model coupled with the engine cluster had not yet been explored in the literature.**"*

### 6.4 File #4 — the 2012 NASA Tech Briefs: what it actually says

**This file is not a paper.** It is a 1-page NASA Tech Briefs (December 2011) issue page. The extraction is scan-mangled and interleaves three unrelated articles. The section headed `Minimum Landing Error Powered-Descent Guidance for Planetary Missions` (attributed to Lars Blackmore and Behcet Acikmese of Caltech for JPL) consists of exactly these sentences (lines 48–49):

> *"An algorithm improves the accuracy with which a lander can be delivered to the surface of Mars. The main idea behind this innovation is the use of a **'lossless convexification,' which converts an otherwise non-convex constraint related to thruster throttling to a convex constraint, enabling convex optimization to be used.** The convexification leads directly to an algorithm that **guarantees finding the global optimum of the original nonconvex optimization problem with a deterministic upper bound on the number of iterations required for convergence.**"*
>
> *"In this innovation, previous work in powered-descent guidance using convex optimization is extended to handle the case where **the lander must get as close as possible to the target given the available fuel, but is not required to arrive exactly at the target.** The new algorithm **calculates the minimum-fuel trajectory to the target, if one exists, and calculates the trajectory that minimizes the distance to the target if no solution to the target exists.** This approach **poses the problem as two Second-Order Cone Programs, which can be solved to global optimality with deterministic bounds on the number of iterations required.**"*

**That is the entire technical content.** No equations, no state vectors, no numbers, no parameters, no comparison to Apollo guidance. The remaining ~80% of the file is two unrelated articles: a "PCS Task Wrapper" software framework for science data processing (lines 30–36), and a CFD base-flow turbulence-model validation note for Marshall Space Flight Center (lines 39–48). Contact/reference metadata: `NPO-46647`, JPL Mail Stop 202-233, Pasadena CA 91109-809.

**The one genuinely useful idea transferable to the user's problem** is the **two-SOCP feasibility fallback**: solve for the exact target first; if infeasible, re-solve minimizing *distance to target* instead. This is **(b) usable as a design pattern** — it maps onto the user's kOS reality as: "if the precision-landing law cannot reach the pad, switch to a nearest-achievable-point law rather than failing." Note that file #3's §6.3.2 final-position relaxation (Eqs. 6.58–6.61) is the modern, softened version of exactly this idea.

### 6.5 Blunt verdict for question 6

**These four files contain no quantitative or qualitative comparison between E-guidance / Apollo polynomial guidance and convex approaches.** The only critique anywhere is file #1's one sentence calling analytic solutions "sub-optimal" and limited in the constraint sets they can handle. Any claim about "E-guidance only satisfies initial and terminal position/velocity constraints" must be sourced from a different document (the user's notes directory contains a file titled exactly that — `2025_分岔法动力下降制导律_指出E制导仅能满足初末位置速度约束.md` — which is outside my assignment).

---

## 7. What is actually usable on kOS — consolidated mapping

The user's scenario: burn start ~10 km, ~650 m/s vertical + ~650 m/s horizontal (≈925 m/s total), net available deceleration only ~50 m/s². For calibration, note file #3's comparable mission: 5 km start, ~219 m/s vertical + ~15 m/s lateral, $T_{\max}/(m\,g)$ = 13.5 (they state *"The maximum thrust corresponds to 13.5 times the initial mass"*), landing at −1 m/s after ~60 s. **The user's scenario is far more demanding** — roughly $925/50 \approx 18.5$ s of pure vertical deceleration time needed, before accounting for gravity losses and the horizontal component.

### 7.1 (a) Directly usable — implementable as-is in a few kOS operations

| Item | Formula | Source | Why it works on kOS |
|---|---|---|---|
| Tilt constraint | $\cos\theta_{\max} \le 1-2(q_2^2+q_3^2)$ | File #1, Eqs. 8–9 | One dot product; no linearization, no solver |
| Glide-slope check | $\|\,[y,z]\,\|_2 \le x\tan\gamma_{gs}$ | Files #2 Eq. 30b, #3 Eqs. 6.10–6.11 | Two multiplies + `SQRT`; a pure feasibility test |
| Terminal-position relaxation pattern | $|y(t_f)|\le s_y$, cone origin shifts: $\|[y-y(t_f),z-z(t_f)]\|_2\le x\tan\gamma$ | File #3, Eqs. 6.58–6.61 | Turns an infeasible target into a reachable one; trivially computable |
| Two-SOCP feasibility fallback *pattern* | min-fuel-to-target, else min-distance-to-target | File #4, lines 48–49 | Design pattern, not an algorithm |
| Unit normalization | $L,V,T=L/V,M$; target all variables $\approx[-1,1]$ | File #3, Table 5.3 | Pure arithmetic hygiene; helps a slow interpreter's numerics |

### 7.2 (b) Usable as a simplified / analytic reduction

| Item | Reduction | Source |
|---|---|---|
| **Bang-bang thrust structure** | Min-time ⇒ both bounds active, bang-coast-bang; min-integral nominal shows "first and only thrust bang". Justifies a **2–3 arc saturation law**: full $T_{\max}$ until a switching condition, then trim. No solver needed — only a switching rule. | File #1 lines 842/846; File #3 line 3414 |
| **Fault/degradation response** | Saturate $T_{\max}$ earlier and longer; +200 kg for one engine of five; +30 kg for 60% degradation of one engine | File #3 lines 4114, 4402 |
| **Ignition timing** | Wait until near terminal velocity, then ignite; SeCO chose PDI at 490.34 m / 86.28 m s⁻¹ vs $v_{terminal}$=85 m s⁻¹ | File #2 line 605 |
| **Free-$t_f$ mechanism** | Time-interval dilation with **~4 scalar phase lengths** $s_k$, bounded $s_{\min}\le s_k\le s_{\max}$ (0.6–10 s), instead of a full adaptive grid. *"we choose to partition the temporal grid based on the phases... This measure is taken to mitigate extreme inter-sample constraint violation."* | File #2 Eqs. 2–3, 21 |
| **$t_f$ initial guess is nearly irrelevant** | 10 guesses spanning 1.0–10.0 UT all converged to $t_f$ within 0.01 UT | File #1 line 843 |
| **Multi-phase via state-triggered constraints** | Single-crossing trigger: $g(x)\ge0$ before, $=0$ at, $\le0$ after. Convex if $g$ and the $c_j$ are convex and $g$ is single-crossing. Altitude trigger is the canonical example. | File #2 Eqs. 13–14 |
| **Warm-start / re-planning** | Initialize each re-plan from slices of the previous trajectory from $t_{fault}$ onward, and **constrain the augmented states to coincide with the currently commanded values** to prevent a discontinuous control jump | File #3 §6.3.4, line 4108 |
| **Attitude out of the guidance loop** | 3-DoF translation + attitude implicitly tracked by an inner controller; keep the trajectory *trimmable* via torque penalties rather than attitude constraints | File #3 lines 1719, 1855 |
| **Never linearize w.r.t. controls** | Keep the system control-affine to avoid high-frequency chatter | File #3 line 2910 |

### 7.3 (c) Offline design insight only

- Every SOCP formulation in §1 — needs an IPM or first-order conic solver. **Not runnable on kOS.**
- State transition matrices $\Phi_A$, the $\Psi$ family, $\bar A_k,\bar B_k,\bar C_k,\bar\Sigma_k,\bar z_k$ — need matrix exponentials/integrals.
- PIPG / xPIPG — first-order, matrix-inverse-free, but still needs $Q,q,H,h$ and $\pi_{\mathcal D}$. Closest to kOS-shaped, still out of reach.
- Trust-region adaptation rules — **not actually given in any of these four files**; only cited ([50], [92]).
- Virtual control / virtual state / slack hierarchies — useful conceptual pattern for "soft constraints plus a penalty," but no solver to host them.
- Transversality $H(t_f)=0$ — needs costates.
- Exact discretization (SeCO Eq. 11) — requires a single-shot nonlinear integration per interval plus STM propagation.

### 7.4 The honest bottom line, stated without speculation

The four documents are a coherent survey of the **convex-optimization school** of powered-descent guidance. Their collective answer to "can I run this on kOS?" is no — and file #3 says so about its own implementation on a desktop MATLAB/CVX/MOSEK stack at 50 Hz, calling it *"not yet online-capable"* because of finite-difference Jacobians and CVX overhead.

What they *do* give the user, and it is not nothing, is a **theoretically-grounded justification for a simple saturation-plus-trim control law** (the bang-bang structure results), a **set of cheap feasibility tests** (glide-slope, tilt from quaternion components, lateral relaxation), a **re-planning protocol** (warm-start from the previous trajectory, continuity-locked initial controls), and a **calibration warning**: the user's 925 m/s at 50 m/s² with a 10 km start is a substantially harder problem than any of the three worked examples in these files, all of which are gentler in both velocity ratio and thrust margin.

---

## 8. Gaps and explicit negative results

Stated plainly, because knowing what is *not* there matters as much as what is:

1. **No lossless-convexification $\sigma$/log-mass formulation** appears in any of the four files. The symbol $\sigma$ in files #1 and #3 means *time dilation*, an unrelated quantity. The LCvx formulation must be obtained from Açıkmeşe & Ploen 2007 or Açıkmeşe/Carson/Blackmore 2013 (both cited in file #1's reference list).
2. **No linearized tilt constraint** appears anywhere. File #1's point is that the quaternion tilt constraint is *already* convex — a stronger and more useful result.
3. **No switch-count theorem and no max-then-min proof.** File #1 reports bang-coast-bang empirically in a *minimum-time* problem. Minimum-fuel structure is not treated.
4. **No adaptive trust-region update rule** is given; both adaptive variants are cited but not reproduced. All three files embed the radius as a variable or a fixed constant.
5. **No E-guidance vs convex comparison.** One sentence in file #1, calling analytic methods sub-optimal and constraint-limited, plus the Klumpp 1974 Apollo citation. Nothing quantitative.
6. **No absolute solve times in file #1** (nondimensional study; timings explicitly deferred to future work) and **no solve times at all in file #3**.
7. **File #4 contains no formulation, no equations, and no numbers** — it is a 1-page Tech Briefs blurb, ~80% of which is two unrelated articles.
8. **No minimum-fuel optimal control derivation** in any file — file #1 and #2 use min-time or max-final-mass objectives; file #3 uses a weighted-integral surrogate.

---

## Reference index (file → section → line numbers consulted)

- `2018_序列凸化六自由度火星火箭动力着陆_自由末时间.md`: Problem 1 (lines 318–350), linearization (361–448), discretization (450–534), trust regions (540–585), virtual control (586–611), Problem 2 (633–681), Algorithm 1 (688–746), Tables 1–2 (753–838), 2-D results (840–843), 3-D results (844–846), conclusion (1090–1094).
- `2022_多阶段火箭着陆序列锥优化实时求解SeCO.md`: time-interval dilation (68–95), linearization (96–97), discretization (98–196), STCs (198–237), virtual state (239–242), conic subproblem (243–301), PIPG (303–324), problem setup (326–410), phase constraints (411–574), discrete subproblem (575–589), results (590–606).
- `2025_TUM硕士论文_多发动机可复用运载器容错着陆制导设计_SCP框架.md`: preliminaries (877–1003), modeling (1279–1728), nominal guidance (1730–1951), scaling (1996–2047), verification (2052–2456), controller (2457–2805), SCP overview (2807–2834), algorithm derivation (2835–3111), Problem 2/3 (3112–3406), nominal validation (3412–3417), Tables 6.1–6.3 (3314–3491), fault cases (4002–4109), fault performance (4111–4659), V&V (4912–5598), limitations (5605–5614).
- `2012_最小着陆误差动力下降制导_行星着陆任务.md`: entire file (lines 1–63); relevant content lines 48–49.
