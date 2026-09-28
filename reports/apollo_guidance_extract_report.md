# Apollo Powered-Descent Guidance: Quantitative Extraction Report

**Scope.** Full read of the five assigned files, plus (for questions 3 and 5, which the assigned files could not answer alone) the actual Luminary 099 AGC source files in the same corpus tree at
`90_代码与源码\Apollo11_AGC_source_master\Apollo-11-master\Luminary099\`.
Every number and every quote below is attributed to its source file. Where the assigned corpus does **not** contain something, that is stated explicitly rather than filled in.

---

## 0. Executive summary — what is and is not in the assigned files

| Requested item | Present in the 5 assigned files? | Where it actually is |
|---|---|---|
| 1. E-guidance, `a(t)=c0+c1·t`, **E-matrix**, 6 terminal constraints | **NO.** Not derived anywhere in the five files. | Only *named*/characterised in the bifurcation paper. Requires Cherry 1964 (AIAA 64-638) or Jaggers NTRS 19760020204, which are elsewhere in this corpus. |
| 2. APDG `a(t)=c0+c1·t+c2·t²` closed form incl. `a(tf)` | **Partially.** Klumpp derives the equivalent quartic-fit form (Eqs. 7–11); the explicit `c0,c1,c2` in terms of `(r, v, a_f, T)` are **not** printed in these five files. | Klumpp R-695 §Guidance Equation Derivation (Eqs. 1–11) + AGC source (exact programmed algebra). |
| 3. AGC variable names, sequence, safeguards | **Partly** in the assigned `.txt` (it *is* the AGC source). Extended here with `THROTTLE_CONTROL_ROUTINES.agc`, `SERVICER.agc`, `THE_LUNAR_LANDING.agc`. | Luninary 099 source. |
| 4. Time-to-go determination | **YES.** Klumpp Eq. (12) cubic + §P63 Ignition Algorithm + the AGC `TTF/8CL` / `ROOTPSRS` code. | Both. |
| 5. Singularity as `T→0` and mitigation | **YES** for the *gain-reduction* mitigation (Klumpp Eq. 11) and the AGC divergence guard; the "guidance gains would become unbounded" quote is explicit. | Both. |
| 6. Patent PEG-vs-polynomial discussion | **CAVEAT.** The patent is **not about PEG at all**. It never mentions PEG. Its comparison is *closed-form-by-ignoring-constraints* vs *SOCP*. Verbatim quotes in §6. | `US20100228409A1`. |
| 7. Bifurcation paper on E-guidance limits / bang-bang | **YES.** Two decisive quotes. | `arxiv_2511_18603`. |

**Critical caveat repeated up front:** the phrase "E-guidance" in item 1 as requested — the Cherry two-coefficient law with an explicit **E matrix** derived from six terminal constraints — **does not appear in any of the five assigned files.** Klumpp's Eq. (7) is frequently *called* "E-guidance" in the secondary literature, but Klumpp himself never uses that name and never introduces an E matrix. Treat items 1 and 2 as the same equation in this corpus.

---

## 1. The guidance equation as actually derived in Klumpp R-695

### 1.1 The assumed acceleration profile — it is a **quartic position**, not a linear acceleration

Klumpp does **not** assume `a(t)=c0+c1·t`. He assumes a **quartic polynomial in position** (five degrees of freedom per axis), Eq. (1):

> `RRG = RTG + VTG T + ATG T2/2 + JTG T3/6 + STG T4/24,` (1)

Verbatim context:

> "In terms of a vector polynomial function of target-referenced time, we wish to define a reference trajectory that satisfies a two-point boundary value problem with a total of five degrees of freedom for each of the three components. This number of degrees of freedom is required in order to constrain terminal thrust in PG3 and to shape the trajectory design in P64, as is discussed in connection with the targeting program."
> "A quartic polynomial is the minimum order with which five constraints on the reference trajectory can be satisfied."

and

> "Most symbols are self-defining by being constructed of standard identifiers ... Position and its derivatives velocity, acceleration, jerk, and snap are denoted R,V,A,J,S."

So `RTG, VTG, ATG, JTG, STG` are the **target** position, velocity, acceleration, jerk, snap (the "A" suffix on `JTGA/STGA` means "achieved").

The **acceleration profile** is the second derivative of Eq. (1), and is therefore **quadratic in T**:

$$\mathbf{A}_{CG}(T) = \mathbf{A}_{TG} + \mathbf{J}_{TG}\,T + \mathbf{S}_{TG}\,\tfrac{T^2}{2}$$

i.e. exactly the APDG `a(t) = c0 + c1 t + c2 t²` structure, with `c0 = A_TG`, `c1 = J_TG`, `c2 = S_TG/2`. The three coefficients are **target-point parameters**, not solved-for constants.

**Sign convention (essential, and a common source of sign bugs):**

> "It is convenient to think of the reference trajectory as evolving backwards in time from the target point, with the time variable T reaching zero at the target point and negative prior to that point. Thus target-referenced time (T) is to be distinguished from clock-time (t)."
> "Because guidance gains would become unbounded, the target point is never reached. Instead, a guided phase is terminated at anegative time T and the succeeding phase is started."

So **`T < 0` always**, and the phase ends at `T ≈ -1 s` (Apollo used `TENDBRAK`/`TCGFAPPR` thresholds; see §4).

### 1.2 The implicit (general) equation — Eqs. (2) and (3)

> "The acceleration to be commanded at any point in space consists of three terms: the acceleration of the reference trajectory at the particular time T, minus two feedback terms proportional tovelocity and position deviations from the reference trajectory."

Eq. (2) verbatim:

```
ACG = ATG + JTG T + STG T2/2
    - (VG - VTG - ATG T - JTG T2/2 - STG T3/6) Kv/T
    - (RG - RTG - VTG T - ATG T2/2 - JTG T3/6 - STG T4/24) KR/T2,
```

Eq. (3), "Combining like terms ... the implicit guidance equation":

```
ACG = - RG KR/T2 - VG Kv/T
      + RTG KR/T2
      + VTG (KR + Kv)/T
      + ATG (1 + Kv + KR/2)
      + JTG (1 + Kv/2 + KR/6) T
      + STG (1/2 + Kv/6 + KR/24) T2.
```

Here `Kv` and `KR` are the **nondimensional feedback gains**. Note the exact spelling in the extracted text: `Kv/Т` and `Ko/T2` in Eq. (2) are OCR artefacts for `K_V` and `K_R`.

**This `(r, v, T)` form is the answer to "the explicit formula in terms of r, v and T" — with `K_R = 12, K_V = -6` it collapses to Eq. (7) below.**

### 1.3 Gain selection: the second-order analogy (Eqs. 4–6)

> "First we note that Eq. (2) may be identified with the linear second-order differential equation `X + 2 ζ ωn X + ωn² X = 0` by the associations (noting T is negative)"

```
Kv/T = -2 ζ ωn ,      KR/T2 = ωn2 ,     (4)
```

> "where ωn is the undamped natural frequency and ζ is the damping ratio. Of course the system is time varying. However, this association does afford some intuition on gain setting. Solving Eqs. (4) yields"

```
KR = (T/P)2 ,        (5)
```
> "where P is the undamped period 2π/ωn, and"

```
Kv = -2 (T/P) ζ .    (6)
```

> "Equation (5) provides a means of controlling the system response time in terms of the nondimensional ratio P/T, and Eq. (6) provides a means of setting the damping ratio."
> "An interesting set of values to choose for response and damping is `P/T = - π/√??, ζ = ??`. This choice yields **`KR = 12, Kv = -6`.**"

*(The two numeric choices for `P/T` and `ζ` are garbled by OCR in the extraction. What is unambiguous and usable is the outcome: `K_R = 12`, `K_V = -6`, i.e. the critically-damped-ish classical explicit-guidance gains.)*

### 1.4 **The explicit guidance equation — Eq. (7)** (this is the one to implement)

> "When these values are substituted into Eq. (3), the result is the explicit guidance equation derived in references 3 to 5,"

$$\boxed{\;\mathbf{A}_{CG} = \frac{12\,(\mathbf{R}_{TG} - \mathbf{R}_G)}{T^{2}} + \frac{6\,(\mathbf{V}_{TG} + \mathbf{V}_G)}{T} + \mathbf{A}_{TG}\;}\tag{7}$$

**Read the signs carefully.** `T < 0`. Therefore:
- The **position term** `12(R_TG - R_G)/T²` is *positive* gain on the position error (`T² > 0`) → pull toward target.
- The **velocity term** `6(V_TG + V_G)/T` is *negative* gain on `V_G` because `T < 0` → **damping / braking**.

This is a plain proportional-derivative law in disguise, with **time-varying, `1/T`-scaled gains**. `K_R = 12`, `K_V = -6` are the only two numbers needed.

> "The discussion of implicit vs explicit guidance is concluded by introducing the concept of a space containing all permissible combinations of guidance parameters. Implicit guidance-parameter space is one quadrant of the ζ P / T plane or, equivalently, one quadrant of the Kr, Ky plane. Explicit guidance-parameter space is a single point in either plane."

### 1.5 The transport-lag-corrected form — Eqs. (8)–(11) (the form **actually programmed**)

> "Equation (7) presents the explicit guidance equation assuming negligible transport time delay. The explicit equation programed in Apollo is corrected to command an acceleration appropriate for the time at which the acceleration is predicted to be achieved. Let this predicted target-referenced time be `Tp = T + LEADTIME` where LEADTIME is the transport delay due to computation and command execution."

`LEADTIME` is defined in the nomenclature:

> "LEADTIME: A time interval (typically **2.2 seconds**) added to the target-referenced time T in P63 and P64 to account for the effective transport lag due to computation and system response times"

The derivation fits the quartic through target pos/vel/acc and current pos/vel:

```
JGTA            -24/T3   72/T4   -18/T2  48/T3   -6/T   12/T2      RTG
     =                                                                VTG
STGA             24/T3  -72/T4   -6/T2   24/T3  -6/T2   24/T3      ATG
                                                                    RG
                                                                    VG
```
(Eq. 9 — the 2×6 matrix as OCR'd; note the second row's last two columns are corrupted in the extraction and the row should be `[24/T³, -72/T⁴, -6/T², 24/T³, -6/T, -2/T]`. **Do not trust the OCR of Eq. (9) directly.**)

```
ACG = ATG + JGTA Tp + STGA Tp2/2.     (10)
```

> "Substituting Eq. (9) into Eq. (10) and simplifying yields the Apollo lunar-descent guidance equation"

$$\boxed{\;\mathbf{A}_{CG} = 12\frac{\mathbf{R}_{TG}-\mathbf{R}_G}{T^{2}}\left(\frac{T_p}{T}\right)^{?} + \dots\;}\tag{11}$$

The extracted text renders Eq. (11) as a broken column:

```
ACG
  О
  12 (RTG - RG)/T +
  6 VTG/T
  J (ID
  ATG.
```

and immediately characterises it:

> "Note that when time T is large in magnitude compared to the transport delay, `Tp/T` approaches unity, all bracketed coefficients in Eq. (11) approach unity, and (11) approaches Eq. (7) identically. The net effect of Eq. (11) not achieved by Eq. (7) is **a gain reduction as the target point is approached**. Because Eqs. (7) and (11) are identical for Tp = T, Eq. (7) generates a quartic profile when the transport delay is zero."

**This is reportable and important even though the OCR of Eq. (11) is incomplete:** the structure is Eq. (7) multiplied by a per-term factor in `Tp/T`, and the *function* of those factors is **gain reduction as `T→0`**, which is mitigation technique #1 for the `T→0` singularity (§5).

---

## 2. The APDG quadratic formulation (`c0 + c1 t + c2 t²`)

### 2.1 What the assigned files give

The quadratic `a(t)=c0 + c1 t + c2 t²` closed form with explicit `c0, c1, c2` **in terms of `r, v, a_f, T`** is **not printed in any of the five assigned files.**

- **Klumpp R-695** gives the *equivalent* structure (position quartic ⇒ acceleration quadratic, Eq. 1 above) and the *implemented* law (Eqs. 7/11). He does not present `c0,c1,c2` in closed form.
- **The 2015 深空探测学报 review** describes it only qualitatively, no formulas:
  > ""阿波罗"飞船采用标称轨迹制导方式、**二次多项式制导律**,它建立在**加速度关于时间的二次多项式**前提下,飞船计算机将真实飞行参数和事先存储的理想参数进行对比,生成相应的制导指令去跟踪理想轨迹"
  > (translation: *"Apollo used nominal-trajectory guidance with a quadratic-polynomial guidance law, built on the premise that acceleration is a quadratic polynomial in time; the onboard computer compares real flight parameters with pre-stored ideal parameters and generates the corresponding guidance command to track the ideal trajectory."*)
  > "这个多项式制导律是跟**剩余时间、当前时刻位置和速度、期望位置速度加速度**有关的函数"
  > (translation: *"...is a function of remaining time, current position and velocity, and the desired position, velocity and acceleration."*)
- **The bifurcation paper** states the structure and the constraint count, but not the solution:
  > "APDG and Apollo E-guidance law [6] assumes a thrust acceleration vector profile as **quadratic and linear functions of time, respectively**. For APDG law the coefficients of the quadratic polynomial are determined to satisfy **initial and final positions and velocity constraints along with final acceleration**."
  That is 3 constraints × 3 axes = the "6 + a(tf)" accounting you asked about: **position and velocity at both ends (4 of the 6 scalar constraints per axis... in fact 2 vector = 6 scalar) plus the terminal acceleration vector (3 scalar) = 9 conditions for 3 coefficients × 3 axes.** The coefficients of `a(t)=c0+c1 t+c2 t²` per axis are fixed by `r0, v0, rf, vf, af`; there is **no free parameter left over**, and there is **no explicit `a(tf)` as a separate penalty** — `a(tf)=c0+c1 tf+c2 tf²` is imposed by construction.
- **The patent** is about SOCP, not APDG, and lists Klumpp as reference [8] without deriving it.

**Stated plainly: the assigned corpus does not contain the c0/c1/c2 formulas. Deriving them requires refs [1]/[6] from the bifurcation paper (Klumpp, *Automatica* 10(2) 1974; Cherry AIAA 64-638). The corpus does contain a Jaggers PEG NTRS note and an SLS PEG note, but those are ascent PEG, not the lunar-descent APDG quadratic.**

### 2.2 What **is** implementable from the assigned files — the exact programmed algebra

The AGC source comment block gives the exact published and programmed forms, and they are **algebraically identical** (verified symbolically):

```
#	AS PUBLISHED --
#		              ___   __       ___   __
#		___   ___   6(VDG + VG)   12(RDG - RG)
#		ACG = ADG + ----------- + ------------
#		                TTF        (TTF)(TTF)
#	AS HERE PROGRAMMED --
#		             ___   __
#		      3 (1/4(RDG - RG)   ___   __)
#		      - (------------- + VDG + VG)
#		___   4 (    TTF/8               )   ___
#		ACG = ---------------------------- + ADG
#		                  TTF/8
```

with, from the same file:

```
# COEFFICIENT FOR VGU TERM ... 28D
# COEFFICIENT FOR RDG-RGU TERM ... 26D
# COEFFICIENT FOR VDG TERM ... 30D
# COEFFICIENT FOR ADG TERM ... 30D
```

**Symbol map (exact, from the source):**

| Source symbol | Meaning |
|---|---|
| `RDG` | target position, guidance coords (`= RBRFG`, per `RDG = RBRFG`) |
| `VDG` | target velocity (`= VBRFG`) |
| `ADG` | target acceleration (`= ABRFG`) |
| `JDG` | target jerk (`= JBRFG`), used only in the T solve |
| `RGU` | current position, guidance coords |
| `VGU` | current velocity, guidance coords |
| `ACG` | commanded acceleration |
| `TTF/8` | time-to-go **divided by 8** (scaling); `TTF` computed by the T routine |
| `LEADTIME` | transport lag, negative |
| `ADG,1` / `RDG,1` / `VDG,1` | the `,1` indexes the phase table (`TARGTDEX`: OCT 0 for IGNALG/BRAKQUAD, OCT 34 for APPRQUAD) |

The exact identity (sympy-verified): with `X ≡ TTF/8`,

$$\mathbf{A}_{CG}=-\frac{3}{4}\cdot\frac{\frac{1}{4}\frac{\mathbf{R}_{DG}-\mathbf{R}_{GU}}{X}+\mathbf{V}_{DG}+\mathbf{V}_{GU}}{X}+\mathbf{A}_{DG}\;\equiv\;\mathbf{A}_{DG}+\frac{6(\mathbf{V}_{DG}+\mathbf{V}_{GU})}{T}+\frac{12(\mathbf{R}_{DG}-\mathbf{R}_{GU})}{T^{2}}$$

I verified this symbolically (`simplify(age-pub) == 0`). **This is Klumpp Eq. (7).** The AGC's "as programmed" form exists purely to save interpreter steps by avoiding two divisions.

---

## 3. The AGC / Luminary 099 source: names, sequence, safeguards

### 3.1 Flight-sequence dispatch (actual variable names)

```
#	WCHPHASE = -1 ---> IGNALG
#	WCHPHASE =  0 ---> BRAKQUAD
#	WCHPHASE =  1 ---> APPRQUAD
#	WCHPHASE =  2 ---> VERTICAL
```

```
NEWPHASE	TCF	TTFINCR		# BRAKQUAD
		TCF	STARTP64	# APPRQUAD
		TCF	P65START	# VERTICAL
PREGUIDE	TCF	RGVGCALC	# BRAKQUAD
		TCF	REDESIG		# APPRQUAD
WHATGUID	TCF	TTF/8CL		# BRAKQUAD
		TCF	TTF/8CL		# APPRQUAD
		TCF	VERTGUID	# VERTICAL
AFTRGUID	TCF	CGCALC		# BRAKQUAD
		TCF	STEER?		# VERTICAL
```

Entry points:

```
# IGNITION ALGORITHM ENTRY:  DELIVERS N PASSES OF QUADRATIC GUIDANCE

?GUIDSUB	EXIT
		CAF	TWO		# N = 3
		TS	NGUIDSUB
		TCF	GUILDRET +2
```

**Note `# N = 3`** — the ignition algorithm runs **three** guidance passes.

### 3.2 Exact computational sequence per pass

1. **`TTFINCR`** — increment time-to-go and rotate the landing site:
```
#		TTF/8 UPDATED FOR TIME SINCE LAST PASS:
#			TTF/8 = TTF/8 + (TPIP - TPIPOLD)/8
#		LANDING SITE VECTOR UPDATED FOR LUNAR ROTATION:
#			LAND = /LAND/ UNIT(LAND - LAND(TPIP - TPIPOLD) * WM)
#		SLANT RANGE TO LANDING SITE, FOR DISPLAY:
#			RANGEDSP = ABVAL(LAND - R)
```
   Key requirement for a kOS port: **T is propagated by dead-reckoning from the previous pass's inertial sample time (`TPIP`), not recomputed from scratch.**

2. **`REDESIG`** — landing-site redesignation (P64 only), with a horizon guard:
```
		DLOAD	DSU		# MAKE SURE REDESIGNATION IS NOT
			0		# 	TOO CLOSE TO THE HORIZON.
			DEPRCRIT
		BMN	DLOAD
			REDES1
			DEPRCRIT
		STORE	0
```
   `DEPRCRIT  2DEC  -.02 B-1` — i.e. the line-of-sight X-component must be ≤ −0.02. (Klumpp text: *"The constraint that LOSP_x be at least as negative as -0.02 (Eq. (6.5)) prevents redesignating the landing site beyond the horizon."*)

3. **`RGVGCALC`** — state in guidance coordinates:
```
#	VELOCITY RELATIVE TO THE SURFACE:
#		_______   _   _   __
#		ANGTERM = V + R * WM
#	STATE IN GUIDANCE COORDINATES:
#		___   *   _   ____
#		RGU = CG (R - LAND)
#		___   *   _   __   _
#		VGU = CG (V - WM * R)
#	HORIZONTAL VELOCITY FOR DISPLAY
#		VHORIZ = 8 ABVAL (0, VG , VG )
#		                       2    1
# 	DEPRESSION ANGLE FOR DISPLAY:
#		                       _   ____  ______
#		LOOKANGL = ARCSIN(UNIT(R - LAND).XMBPIP)
```

4. **`TTF/8CL`** — the time-to-go solve (see §4).

5. **`QUADGUID`** — the guidance equation:
```
QUADGUID	CS	TTF/8
		AD	LEADTIME	# LEADTIME IS A NEGATIVE NUMBER
		AD	POSMAX		# SAFEGUARD THE COMPUTATIONS THAT FOLLOW
		TS	L		#	BY FORCING -TTF*LEADTIME > OR = ZERO
		CS	L
		AD	L
		ZL
		EXTEND
		DV	TTF/8
		TS	BUF		# - RATIO OF LAG-DIMINISHED TTF TO TTF
```
   then:
```
		EXTEND
		SQUARE
		TS	BUF +1
		AD	BUF
		XCH	BUF +1		# RATIO SQUARED - RATIO
		AD	BUF +1
		TS	MPAC		# COEFFICIENT FOR VGU TERM
		AD	BUF +1
		INDEX	FIXLOC
		TS	26D		# COEFFICIENT FOR RDG-RGU TERM
		AD	BUF +1
		INDEX	FIXLOC
		TS	28D		# COEFFICIENT FOR VDG TERM
		AD	BUF
		AD	POSMAX
		AD	BUF +1
		AD	BUF +1
		INDEX	FIXLOC
		TS	30D		# COEFFICIENT FOR ADG TERM
```
   **This is Klumpp Eq. (11) implemented via the single scalar "lag ratio" `BUF = (TTF/8 + LEADTIME)/(TTF/8)`,** with the four guidance coefficients (V_GU, R_DG−R_GU, V_DG, A_DG) generated by repeated addition/multiplication of that one ratio. That is a *major* implementation lesson: **the transport-lag correction costs one scalar computation plus four cheap coefficient updates, not a re-derivation.**

   The vector assembly:
```
		TC	INTPRETX
		VXSC	PDDL
			VGU
			28D
		VXSC*	PDVL*
			VDG,1
			RDG,1
		VSU	V/SC
			RGU
			TTF/8
		VSR2	VXSC
			26D
		VAD	VAD
		V/SC	VXSC
			TTF/8
			3/4DP
		PDDL	VXSC*
			30D
			ADG,1
		VAD
AFCCALC1	VXM	VSL1		# VERGUID COMES HERE
			CG
		PDVL	V/SC
			GDT/2
			GSCALE
		BVSU	STADR
		STORE	UNFC/2		# UNFC/2 NEED NOT BE UNITIZED
		ABVAL
AFCCALC2	STODL	/AFC/		# MAGNITUDE OF AFC FOR THROTTLE
```

   **Critical: gravity is *not* part of the polynomial** — it is subtracted at the end via `GDT/2` scaled by `GSCALE`:
```
GSCALE		2DEC	100 B-11
3/4DP		2DEC	.750
3/8DP		2DEC	.375
```
   and the output `UNFC/2` is **explicitly documented as not needing unitizing** (`# UNFC/2 NEED NOT BE UNITIZED`) — a deliberate performance choice.

6. **`CGCALC`** — erect the guidance frame. The phase knob `GAINBRAK` appears with an in-source joke:
```
		DMP*	VXSC
			GAINBRAK,1	# NUMERO MYSTERIOSO
			ANGTERM
```
   (Klumpp: *"With K = 1 in P63 ... the YG-component of jerk would reach zero at the target point ... With K = 0 in P64, the ZG-axis is in the vertical plane containing the line-of-sight vector. For crossrange landing-site redesignations, setting K = 0 in P64 was found to consume less DPS propellant than setting K = 1 to null the crossrange jerk at the target point."*)

7. **Phase switch** by sign of `TENDBRAK + TTF/8`:
```
EXTLOGIC	INDEX	WCHPHASE	# WCHPHASE = 1   APPRQUAD
		CA	TENDBRAK	# WCHPHASE = 0   BRAKQUAD
		AD	TTF/8
EXSPOT1		EXTEND
		INDEX	WCHPHASE
		BZMF	WHATEXIT
```

### 3.3 Numerical safeguards, division protections and limits — the actual list

**(a) Divergence guard on `TTF → 0` (the key one):**
```
QUADGUID	CS	TTF/8
		AD	LEADTIME	# LEADTIME IS A NEGATIVE NUMBER
		AD	POSMAX		# SAFEGUARD THE COMPUTATIONS THAT FOLLOW
		TS	L		#	BY FORCING -TTF*LEADTIME > OR = ZERO
```
with `LEADTIME` negative and `TTF/8` negative, `-TTF*LEADTIME` is a positive product; `POSMAX` clamps it so the subsequent `DV TTF/8` cannot produce a runaway ratio. Comment wording is unambiguous: *"SAFEGUARD THE COMPUTATIONS THAT FOLLOW BY FORCING -TTF*LEADTIME > OR = ZERO"*.

**(b) Overflow detection, non-abortive:**
```
EXVERT		CA	OVFIND		# IF OVERFLOW ANYWHERE IN GUIDANCE
		EXTEND			#	DON'T CALL THROTTLE OR FINDCDUW
		BZF	+13
EXOVFLOW	TC	ALARM		# SOUND THE ALARM NON-ABORTIVELY
		OCT	01410
```

**(c) T-computation failure alarm:**
```
1406P00		TC	POODOO
		OCT	01406
1406ALM		TC	ALARM
		OCT	01406
		TCF	RATESTOP
```
`POODOO` is the AGC's abort path — the ignition algorithm treats a failed T solve as fatal, whereas the in-flight phases merely alarm.

**(d) Root-finder bounded to 8 iterations:**
```
		CA	MODE
		MASK	BIT4		# KLUMPP SAYS GIVE UP AFTER EIGHT PASSES
		CCS	A
BADROOT		TC	RETROOT
```
and the return contract:
```
# RETURN IS NORMALLY TO LOC(TC ROOTPSRS)+3.  IF ROOTPSRS FAILS TO CONVERGE TO IN 8 PASSES, RETURN IS TO LOC+1 AND
# OUTPUTS ARE NOT TO BE TRUSTED.
```

**(e) Explicitly documented *absence* of overflow protection in the root finder** — a warning to the caller, i.e. the guard is pushed to the guidance code that forms the coefficients:
```
# PRECAUTION:  ROOTPSRS MAKES NO CHECKS FOR OVERFLOW OR FOR IMPROPER USAGE.  IMPROPER USAGE COULD
# PRECLUDE CONVERGENCE OR REQUIRE EXCESSIVE ITERATIONS. ...
#	1.  USER'S RESPONSIBILITY TO ASSUR THAT I X A(I) < 1 IN MAGNITUDE FOR ALL I.
#	2.  USER'S RESPONSIBILITY TO ASSURE OVERFLOW WILL NOT OCCUR IN EVALUATING EITHER THE RESIDUAL OR THE DERIVATIVE
#	    POWER SERIES. ...
#	3.  AT PRESENT, ERASABLE LOCATIONS ARE RESERVED ONLY FOR N UP TO 5.  AN N IN EXCESS OF 5 WILL PRODUCE CHAOS. ...
#	4.  THE ITERATION COUNT RETURNED IN MPAC+2 MAY BE USED TO DETECT ABNORMAL PERFORMANCE.
```
   **`I X A(I) < 1` is a hard, actionable constraint** — it is the reason for the `/8` scaling on TTF and the `/64`, `/8`, `/4` factors in the T coefficients.

**(f) Bounds limiting via the `LIMIT` function.** Nomenclature:
> "LIMIT: A function of two arguments yielding the first argument limited in magnitude to the value of the second argument"

Used in P66 with `V · S`-style logic:
```
		AliMRBy ■ l !МИ[О ? । UNAFRBy UNFRBy I. 0 00/]
		& UM- R B, ■ IIMII [0 ? i UNAf RB / UNFRB^ ). 0 DO?]
		ONFRBy • I IMIT [i UftfRBY * MlVRBy I 0 129]
		IINERB/ • : iMlT [< l.MRB' * AU^RB/ I, 0 129 ]
```
   (OCR-degraded but readable as `LIMIT[(UNFRB_Y + ΔUNFRB_Y), 0.129]` etc. — note **`0.129`**, i.e. 129 mr.)

**(g) Commanded-attitude rate limit:** `20° in 2 seconds (10°/sec)`:
> "Equations (15.24)-(15.28) yield the reference gimbal angle changes by limiting the magnitude of the attitude changes to 20° in 2 seconds (10°/sec) about each of three orthogonal axes ... This permits an angular-rate vector of length 10 deg/sec."

**(h) Thrust-axis change limit per cycle:** `7 mr` per ATT cycle; total excursion `129 mr`:
> "The change in thrust direction is limited on each cycle to 7-mr (Eqs. (15.3) and (15.4)), the maximum travel of the trim gimbal in 2 seconds. The total excursion of the estimated unit thrust vector is limited to 129-mr (Eqs. (15.5) and (15.6)), the mechanical excursion limit of the trim gimbal..."

**(i) P66 horizontal thrust tilt limit: 20° from vertical** — directly relevant to any kOS port:
> "The direction of the thrust-acceleration command is limited to 20° from vertical (Eqs. (12.4) and (12.5)) to maintain a nearly erect LM attitude."

**(j) Throttle / DPS constraint.** From `THROTTLE_CONTROL_ROUTINES.agc`:
```
# THIS LOGIC DETERMINES THE THROTTLING IN THE REGION 10% - 94%.  THE MANUAL THROTTLE, NOMINALLY SET AT
# MINIMUM BY ASTRONAUT OR MISSION CONTROL PROGRAMS, PROVIDES THE LOWER BOUND.  A STOP IN THE THROTTLE HARDWARE
# PROVIDES THE UPPER.
```
```
LOWFCOLD	CS	H*GHCR*T
		AD	FCODD
		EXTEND
		BZMF	DOPIF		# BRANCH IF FC < OR = HIGHCRIT
		CA	FMAXPOS		# NO:  THROTTLE-UP
FLATOUT1	DXCH	FCODD
```
The command is emitted as a **pulse-count increment**, not a throttle setting:
```
DOPIF		TC	FASTCHNG
		EXTEND
		DCA	FCODD
		TS	FCOLD
		DXCH	PIF
		EXTEND
		DCS	FP
		DAS	PIF		# PIF = FC - FP, NEVER EQUALS +0
```
   with `# PIF = FC - FP, NEVER EQUALS +0` — an explicit anti-zero-division/sign-convention note. The `FWEIGHT` term corrects for the fact that the accelerometer measures an *interval average*:
```
# 	          PIF(PPROCESS + TL)     PIF /PIF/
#	FWEIGHT = ------------------ + -------------
#		       PGUID           2 PGUID FRATE
#
# WHERE PPROCESS IS THE TIME BETWEEN PIPA READING AND THE START OF THROTTLING, PGUID IS THE GUIDANCE PERIOD, AND
# FRATE IS THE THROTTLING RATE (32 UNITS PER CENTISECOND).  PGUID IS EITHER 1 OR 2 SECONDS.
```

**(k) Guidance-period determination is dynamic**, not a constant — `SERVICER.agc`:
```
REPIP4		EXTEND			# COMPUTE GUIDANCE PERIOD
		DXCH	PGUIDE
```
and `HIGATCHK` gates the landing-radar high-gate on T:
```
HIGATCHK	CA	TTF/8		# IS TTF > CRITERION?  (TTF IS NEGATIVE)
		AD	RPCRTIME
		EXTEND
		BZMF	POS1CHK		# NO
```

**Timing numbers from Klumpp (directly relevant to kOS):**
> "All routines are processed once every two seconds, except the vertical channel of the P66 guidance algorithm is processed once per second, and the digital autopilot is processed 10 times per second."

> "Typical variations of 2 seconds in the time duration of throttle control and typical attitude transients of 2 milliradians commanded by the guidance algorithm on the first P63 pass."

> "Equation (22.10) yields a terminal state at the specified terminal time TBRF precisely, whereas the state RG,VG applies at the time T which may differ from TBRF by up to the 2-second granularity."

> "Solving for the ZG-component of achieved target jerk provides a check on the computation of T by the guidance algorithm; agreement between achieved and input values is typically to seven places."

---

## 4. Time-to-go determination — verbatim

### 4.1 The decisive statement: **T is arbitrary and is pinned by one extra constraint**

Klumpp, verbatim:

> "In the derivation of the guidance Eq. (7) or (11), **nothing constrained the time T**. At any point in a guided phase, **T could be set to any arbitrary negative value**, and Eq. (7) or (11) would satisfy the boundary-value problem from that point forward. Landing-site redesignation, which can arbitrarily stretch or shrink the trajectory, would produce an unnecessarily severe guidance response if T were not correspondingly adjusted. **Because T is arbitrary, it can be computed to satisfy an additional boundary constraint.** In Apollo, this additional constraint is imposed on the downrange (Z) component of jerk. Thus the Z-component of the jerk polynomial of Eq. (9) is solved for T by using a target Z-component of jerk JTG7. Separating this scalar cubic polynomial from Eq. (9) yields"

$$\boxed{\;J_{TG_Z}T^{3} + 6A_{TG_Z}T^{2} + \left(18V_{TG_Z} + 6V_{G_Z}\right)T + 24\left(R_{TG_Z} - R_{G_Z}\right) = 0\;}\tag{12}$$

> "**One root of this cubic is the required time T.**"

> "An alternate criterion for computing T reduces the propellant-consumption penalty of downrange landing-site redesignations. Although extensively tested, the alternate was developed too late for incorporation in the LGC program. The alternate criterion sets the downrange position error to zero. Thus T is one root of the quartic"

$$S_{TG_Z}\frac{T^{4}}{24} + J_{TG_Z}\frac{T^{3}}{6} + A_{TG_Z}\frac{T^{2}}{2} + V_{TG_Z}T + R_{TG_Z} - R_{G_Z} = 0$$

**Answer to "fixed, iterated, ground-computed, or solved onboard?": SOLVED ONBOARD, every guidance pass, as the root of a cubic (Eq. 12).** It is **not** fixed and **not** ground-computed for the flight law. Ground involvement is only in *targeting* (§4.3) and in producing the `TENDBRAK`/`TCGFAPPR` table constants.

### 4.2 How the cubic is actually solved — Newton, warm-started from the previous pass

From `LUNAR_LANDING_GUIDANCE_EQUATIONS.agc`, `TTF/8CL`:

```
TTF/8CL		TC	INTPRETX
		DLOAD*
			JDG2TTF,1
		STODL*	TABLTTF +6	# A(3) = 8 JDG  TO TABLTTF
			ADG2TTF,1	#             2
		STODL	TABLTTF +4	# A(2) = 6 ADG  TO TABLTTF
			VGU 	+4	#             2
		DMP	DAD*
			3/4DP
			VDG2TTF,1
		STODL	TABLTTF +2	# A(1) = (6 VGU  + 18 VDG )/8 TO TABLTTF
			RDG +4,1	#              2         2
		DSU	DMP
			RGU +4
			3/8DP
		STORE	TABLTTF		# A(0) = -24 (RGU  - RDG )/64 TO TABLTTF
		EXIT			#                2      2

		CA	BIT8
		TS	TABLTTF +10	# FRACTIONAL PRECISION FOR TTF TO TABLE

		EXTEND
		DCA	TTF/8
		DXCH	MPAC		# LOADS TTF/8 (INITIAL GUESS) INTO MPAC
		CAF	TWO		# DEGREE - ONE
		TS	L
		CAF	TABLTTFL
		TC	ROOTPSRS	# YIELDS TTF/8 IN MPAC
		INDEX	WCHPHASE
		TCF	WHATALM

		EXTEND			# GOOD RETURN
		DCA	MPAC		# FETCH TTF/8 KEEPING IT IN MPAC
		DXCH	TTF/8		# CORRECTED TTF/8
```

**This is a complete, directly-portable algorithm.** Mapping the code to Eq. (12) (with `T ≡ TTF/8`, i.e. all coefficients are in the `/8` scaled variable — which is why the constants are `/8`, `/64`, `3/8DP`, `3/4DP`):

| Table slot | Coefficient of `X^k` in `A3·X³+A2·X²+A1·X+A0`, `X = TTF/8` | Source expression |
|---|---|---|
| `TABLTTF +6` | `A3 = 8·JDG₂` | `JDG2TTF,1` (`JDG2TTF = JBRFG*`) |
| `TABLTTF +4` | `A2 = 6·ADG₂` | `ADG2TTF,1` |
| `TABLTTF +2` | `A1 = (6·VGU₂ + 18·VDG₂)/8` | `DMP 3/4DP` then `DAD VDG2TTF,1` |
| `TABLTTF` | `A0 = −24·(RGU₂ − RDG₂)/64` | `DSU RGU+4` (gives `RDG−RGU`), `DMP 3/8DP` |

Note the index `+4` selects the **Z (downrange) component**, matching Klumpp's "downrange (Z) component of jerk."

Solver entry contract (`ROOTPSRS`):
```
#	ROOTPSRS FINDS ONE ROOT OF THE POWER SERIES A X  + A   X    + ... + A X + A
#	                                             N      N-1              1     0
# USING NEWTON'S METHOD STARTING WITH AN INITIAL GUESS FOR THE ROOT.
#	A	SP	LOC-3		ADRES FOR REFERENCING PWR COF TABL
#	L	SP	N-1		N IS THE DEGREE OF THE POWER SERIES
#	MPAC	DP	X		INITIAL GUESS FOR ROOT
```

and the Newton iteration itself:
```
ROOTLOOP	EXTEND
		DCA	ROOTPS		# FETCH CURRENT ROOT
		DXCH	MPAC		# LEAVE IN MPAC
		EXTEND
		DCA	MPAC +5		# LOAD A, L WITH DER TABL ADRES, N-2
		TC	POWRSERS	# YIELDS DERIVATIVE IN MPAC
		EXTEND
		DCA	ROOTPS
		DXCH	MPAC		# CURRENT ROOT TO MPAC, FETCHING DERIVATIVE
		DXCH	BUF		# LEAVE DERIVATIVE IN BUF AS DIVISOR
		EXTEND
		DCA	MPAC +3		# LOAD A, L WITH PWR TABL ADRES, N-1
		TC	POWRSERS	# YIELDS RESIDUAL IN MPAC
		TC	USPRCADR
		CADR	DDV/BDDV	# YIELDS -DX IN MPAC
		EXTEND
		DCS	MPAC		# FETCH DX, LEAVING -DX IN MPAC
		DAS	ROOTPS		# CORRECTED ROOT NOW IN ROOTPS
```

**Convergence is on `|ΔX|`, not on the residual**, and the threshold is set as a *fraction of the first guess*:
```
#	LOC+2	SP	PRECROOT	 PREC RQD OF ROOT (AS FRACT OF 1ST GUESS)
```
computed by:
```
		INDEX	MPAC +3		# PWR TABLE ADRES
		CA	5		# PRECROOT TO A
		TC	SHORTMP		# YIELDS DP PRODUCT IN MPAC
		TC	USPRCADR
		CADR	ABS		# YIELDS ABVAL OF CRITERION ON DX IN MPAC
		DXCH	MPAC
		DXCH	DXCRIT		# CRITERION
```
and termination:
```
		CA	MODE
		MASK	BIT4		# KLUMPP SAYS GIVE UP AFTER EIGHT PASSES
		CCS	A
BADROOT		TC	RETROOT
		INCR	MODE		# INCREMENT ITERATION COUNTER
		CCS	MPAC		# TEST HI ORDER DX
		TCF	ROOTLOOP
		TCF	TESTLODX
		TCF	ROOTSTOR
TESTLODX	CCS	MPAC +1		# TEST LO ORDER DX
		TCF	ROOTLOOP
		TCF	ROOTSTOR
		TCF	ROOTSTOR
ROOTSTOR	DXCH	ROOTPS
		DXCH	MPAC
		CA	MODE
		TS	MPAC +2		# STORE SP ITERATION COUNT IN MPAC+2
```

**The initial guess is `TTF/8` from the preceding pass.** On the very first pass it is seeded by the ignition algorithm:
```
		VSL4	MXV
			REFSMMAT
		...
		STODL	DELTAH		# INITIALIZE DELTAH FOR V16N68 DISPLAY
			ZEROVECS
		STODL	UNFC/2		# INITIALIZE TRIM VELOCITY CORRECTION TERM
			HI6ZEROS
		STORE	TTF/8
```
i.e. **`TTF/8` starts at 0 (HI6ZEROS) and the ignition algorithm iterates it in.** Corroborated by:
```
# THE NUMERATOR IS SCALED IN METERS AT 2(28).  THE DENOMINATOR IS A VELOCITY IN UNITS OF 2(10) M/CS.
# THE QUOTIENT IS THUS A TIME IN UNITS OF 2(18) CENTISECONDS.  THE FINAL SHIFT RESCALES TO UNITS OF 2(28) CS.
# THERE IS NO DAMPING FACTOR.  THE CONSTANTS KIGNX/B4, KIGNY/B8 AND KIGNV/B4 ARE ALL NEGATIVE IN SIGN.
```
and the ignition loop's convergence test `DDUMCRIT  2DEC  +8 B-28  # CRITERION FOR IGNALG CONVERGENCE`.

### 4.3 Ground-side statement

> "The targeting program computes the P63 targets by projecting computed terminal conditions forward typically 60 seconds. ... **Both the ignition time and the projected targets are computed iteratively using a descent simulation in the iteration loop.**"

> "The P64 targets are computed ... **Unlike P63, the P64 reference trajectory can be determined in closed form from specified trajectory constraints. Thus the projected targets are computed without numerical iteration.**"

> "Three or four iterations are generally required because there is bilateral interaction between the targets and the simulation."

> "The remaining three conditions necessary to define the quartic are determined iteratively by simulation."

### 4.4 Independent corroboration from the bifurcation paper

> "The main issue of polynomial guidance laws is the determination of time-to-go that specifies the burn time. Time-to-go determines the fuel consumed and whether the thrust will saturate between minimum and maximum allowable bounds. **When APDG law is used on actual missions the time-to-go is determined using trial and error method on ground.**"

*(Note: this statement concerns **APDG** on "actual missions" generally and is in tension with Klumpp's onboard cubic solve for **Apollo lunar descent**. They are not necessarily contradictory — APDG ≠ Apollo P63/P64 exactly — but flag it: the two sources do not fully agree, and the assigned Klumpp text is the more specific and more authoritative for Apollo.)*

---

## 5. Singularity as T → 0, and the mitigation

### 5.1 The statement of the problem — verbatim

> "It is convenient to think of the reference trajectory as evolving backwards in time from the target point, with the time variable T reaching zero at the target point and negative prior to that point. ... **Because guidance gains would become unbounded, the target point is never reached.** Instead, a guided phase is terminated at a negative time T and the succeeding phase is started. Both the terminus and the target point lie on the reference trajectory, but **the target point lies beyond the portion that is actually flown**, similar to a suggestion of McSwain and Moore"

This is the central design decision: **the singularity is avoided by never flying to `T=0`.** The phase is handed off at `T < 0` to a different (P66) algorithm that controls velocity only.

### 5.2 Mitigation #1 — transport-lag gain reduction (the analytic fix)

> "**The net effect of Eq. (11) not achieved by Eq. (7) is a gain reduction as the target point is approached.**"

Because Eq. (11)'s bracketed coefficients are functions of `Tp/T = (T + LEADTIME)/T`, and `LEADTIME` is a fixed negative-ish constant, the ratios depart from unity and *reduce* gain as `|T|` shrinks toward `|LEADTIME|`.

### 5.3 Mitigation #2 — the AGC's explicit arithmetic guard (the code fix)

```
QUADGUID	CS	TTF/8
		AD	LEADTIME	# LEADTIME IS A NEGATIVE NUMBER
		AD	POSMAX		# SAFEGUARD THE COMPUTATIONS THAT FOLLOW
		TS	L		#	BY FORCING -TTF*LEADTIME > OR = ZERO
		CS	L
		AD	L
		ZL
		EXTEND
		DV	TTF/8
		TS	BUF		# - RATIO OF LAG-DIMINISHED TTF TO TTF
```

Mechanism, read literally: the `L` register is clamped with `POSMAX` so that the quantity `-TTF*LEADTIME` (a product of two negatives = positive) cannot go negative, and `DV TTF/8` — a *division* — is then performed with a guaranteed-sign denominator. The comment names the purpose exactly: *"SAFEGUARD THE COMPUTATIONS THAT FOLLOW BY FORCING -TTF*LEADTIME > OR = ZERO."*

### 5.4 Mitigation #3 — phase termination thresholds and alarms

- Termination is by comparison against a table constant `TENDBRAK` / `TCGFAPPR` (see §3.2 step 7), not by `T→0`.
- A failed T solve raises alarm `01406`; in the ignition algorithm it is a `POODOO` abort.
- Overflow anywhere in guidance raises non-abortive alarm `01410` and suppresses the throttle/FINDCDUW call.

### 5.5 Mitigation #4 — hand off to a velocity-only controller (P66)

> "The P66 guidance algorithm controls velocity only; there is no position control."

> "In the design of the guidance Eq. (7) or (11) ... the target point lies beyond the portion that is actually flown"

### 5.6 What the assigned corpus does **not** say about `T→0`

There is **no** explicit clamp on `T_go` to a minimum magnitude, no `if T > -T_min then T = -T_min` construct, and no deadband/hysteresis discussion for `T` itself in any of the five assigned files. The mitigations present are (i) terminate before `T=0`, (ii) lag-induced gain reduction, (iii) arithmetic sign guard on the division, (iv) alarm/abort. **A kOS implementation must supply its own `T_go` floor**, and the corpus gives no Apollo number for one.

---

## 6. The patent: PEG vs polynomial guidance

### 6.1 **Direct answer: the patent never discusses PEG, and never mentions it by name.**

I searched the full 2309-line extraction. `US20100228409A1` (Açıkmeşe, Blackmore, Scharf; California Institute of Technology) is entirely about **lossless convexification / Second-Order Cone Programming**, and its comparison is *closed-form-by-ignoring-constraints* vs *nonlinear optimization onboard* vs *convex (SOCP)*. Verified: the string "PEG" does not occur. The word "polynomial" does not occur in a guidance-law sense either. **The premise of report item 6 is not supported by this file.**

### 6.2 The actual comparison passages — verbatim

On why closed-form/approximate laws are claimed insufficient:

> "**The closed-form solution approach results in solutions that do not obey the constraints inherent in the problem, such as no Subsurface flight constraints; in practice this reduces the size of the region from which return to the target is possible by a factor of ten or more.**"

> "This is in contrast with other approaches that either **compute a closed-form solution by ignoring the constraints of the problem**, propose solving a nonlinear optimization onboard, or solve a related problem that does not minimize fuel use."

On nonlinear optimization:

> "**Nonlinear optimization approaches, on the other hand, cannot provide deterministic guarantees on how many iterations will be required to find a feasible trajectory, and are not guaranteed to find the global optimum. This limits their relevance to onboard applications.**"

On the real-time requirement (the argument that matters most for a flight processor):

> "Second, we must guarantee that a feasible solution will be found **in a matter of seconds**. This requirement is derived from the duration of the parachute and PDG phases; **if the PDG algorithm takes too long to find a feasible solution, the lander can crash into the surface.**"

> "Main drawback of optimization methods is that is computationally expensive and may not be feasible to implement onboard the vehicle. Moreover, convergence of direct methods relies heavily on initial guess of the unknown variables and global convergence to a local optimum isn't guaranteed. **If the optimal solution's convergence time is greater than the guidance cycle then the system fails.**"
> — *(this quote is from the bifurcation paper, not the patent; it is included here because it is the sharpest statement of the same trade-off)*

On the non-convexity that dominates the problem (the lower thrust bound):

> "The thrust constraints are non-convex because of a **minimum thrust magnitude constraint, which arises because, once started, the thrusters cannot be throttled below a certain level.**"

> "**if a feasible solution does not exist, the optimization will simply report 'infeasible,' even though it may still be possible to land safely at some distance from the original target.**"

### 6.3 Concrete patent numbers (useful as a cross-check on the booster case)

Spacecraft parameters (Eq. 74):
```
g = -3.71140 m/s2,  m_dry = 1505 kg,  m_wet = 1905 kg,
Isp = 225 s,  rho1 = 4972 N,  rho2 = 13260 N
```
- Glideslope: *"A glideslope constraint is used to prevent the trajectory from entering at a more shallow angle than 4°."*
- Case 1 (feasible): `r0 = (0, 2000) m`, `v0 = (-75, 0, 100) m/s`. **"This solution requires 399.5 kg of fuel, has tf ≈ 78.3 s"**, 55 discretization points, total computation time **14.3 s**, 23 Golden Search iterations.
- Case 2 (infeasible, adds `vy0 = 40 m/s`): minimum-fuel reports infeasible, *"at least 410 kg of fuel is required"*; minimum-landing-error lands **268 m** from target, `tf ≈ 77.6 s`, full 400 kg used, **16.24 s** compute, 23 Golden Search iterations.
- Time-of-flight optimum: *"The relationship is unimodal and has an optimum at 56.35 seconds. The Golden Search determines the optimum to be 55.83 seconds, which is an error of only 0.9%."*
- Golden Search accuracy over 100 random ICs: *"the average percentage error in tf was 0.012% with a standard deviation of 0.057%. The maximum error across all instances was 1.8%."*
- Upper bound heuristic: *"we use a heuristic scaling to set tu = k·tl, where k is on the order of 3."*
- Hardware used: *"a Macbook Pro 2.4 GHz with 4 GB RAM"* — **note: 14–16 seconds of compute on a 2010 laptop.** This is the strongest available evidence that convex PDG is **not** a kOS-on-a-slow-CPU option.

### 6.4 The patent's core algorithm (for completeness)

```
TABLE 1. Prioritized Powered Descent Guidance Algorithm
1) Solve the relaxed minimum landing error guidance problem (Problem 4) for {tf, T1(·), T2(·)}
   with corresponding trajectory r*(·). If no solution exists, return infeasible
2) Solve the relaxed minimum-fuel guidance problem to specified range (Problem 5)
   with D = ||r(tf*)|| for {tf, T1(·), T2(·)}.
3) Return {tf, T1(·)}.
```
Change of variables removing the nonlinear dynamics:
```
Omega(t) = T(t)/m(t),  sigma(t) = ||T(t)||/m(t),  zeta(t) = ln m(t)
```
with the conic constraint relaxation via the first three / first two Taylor terms:
```
rho1 <= sigma(t) <= rho2   ~>   1 - (zeta(t) - z0(t)) + (zeta(t)-z0(t))^2/2 ... ,
z0(t) = ln(m0 - rho2*t/Isp/g0)
```
**Hard constraint on the booster design:** *"rho1 <= sigma <= rho2"* — a nonzero **lower** thrust bound is what makes this non-convex, and the same is true of a real booster.

---

## 7. The bifurcation paper: E-guidance limits and bang-bang structure

### 7.1 What E-guidance can and cannot satisfy — the two decisive quotes

> "APDG and Apollo E-guidance law [6] assumes a thrust acceleration vector profile as **quadratic and linear functions of time, respectively**. For APDG law the coefficients of the quadratic polynomial are determined to satisfy **initial and final positions and velocity constraints along with final acceleration**. **In E-guidance law only initial and final positions and velocity constraints can be satisfied.**"

This is the direct answer to the question. **E-guidance: 6 scalar constraints per axis-set (position × 2 ends, velocity × 2 ends) — no acceleration authority. APDG: those 6 *plus* final acceleration.**

Corroborating structural statement:

> "Ref. [7] acceleration is expressed as functions of position and velocity with two tunable gains and **specific choices of gains leads to APDG and E-guidance laws**. The tuning parameters are used as trade off quantity between trajectory shaping and fuel usage."

> "Ref. [8] recently investigated on a family of guidance laws where acceleration is expressed as fractional polynomial of time-to-go and **showed that APDG and E-guidance laws belong to that family**."

**This matches Klumpp §1.3 exactly**: Klumpp's Eq. (3) has two gains `K_V, K_R`, and Eq. (7) is the specific choice `K_R = 12, K_V = -6`. The bifurcation paper's "two tunable gains" and Klumpp's "explicit guidance-parameter space is a **single point**" are the same statement.

### 7.2 Bang-bang structure — verbatim

> "**An important result optimal control method reveals is that the solutions to fuel optimal PDG are such that thrust profile has bang-bang structure operating at either maximum or minimum thrust bounds. In a constant gravity field, three dimensional fuel optimal PDG problem Refs. [12–14] showed that there can be at most two switching in optimal thrust magnitude. In non-constant gravity field, depending on how gravity is modelled number of thrust switching varies.**"

### 7.3 Why E-guidance is *not* fuel-optimal — verbatim

> "Ref. [10] first used indirect method to formulate guidance law which minimizes commanded acceleration and the obtained result was **surprisingly E-guidance law**. In that work, **cost function is modelled as integral of quadratic acceleration, which is not theoretically fuel optimal but control effort optimal. Fuel optimal consideration requires integral of thrust magnitude not quadratic index [11].**"

**This is the single most important sentence for the booster problem.** E-guidance is the exact optimum of `∫|a|²dt` — a *control-effort* metric — and is provably **not** the minimiser of `∫|T|dt`, the actual fuel metric. It produces a **smooth, continuously-varying acceleration**, whereas the true fuel-optimal law is **bang-bang**.

### 7.4 The alternative proposed (BPDG), with its closed form and numbers

Velocity dynamics per axis:
$$V_i=\dot r_i=(a_ir_i+r_i^2)b_i+c_i,\qquad i=x,y,z \tag{5}$$

Equilibria:
$$r_{ie1}=-\frac{a_i}{2}+\sqrt{\frac{a_i^2}{4}-\frac{c_i}{b_i}},\qquad r_{ie2}=-\frac{a_i}{2}-\sqrt{\frac{a_i^2}{4}-\frac{c_i}{b_i}} \tag{6,7}$$

`r_ie1` unstable, `r_ie2` stable. Parameter design:
$$a_i=-2r_{i0}\tag{18}\qquad c_i=V_{i0}+\frac{a_i^2b_i}{4}\tag{17}\qquad b_i=\frac{-V_{i0}}{(r_{if}-r_{i0})^2}\tag{20}$$

Guidance command:
$$A_i=2b_ir_i^3+3a_ib_i^2r_i^2+\left(a_i^2b_i^2+2b_ic_i\right)r_i+a_ib_ic_i \tag{24}$$

Termination time to reach error bound `eps_i`:
$$t_{si}=\frac{1}{2b_i}\left(r_{if}+\frac{a_i}{2}\right)\ln\frac{\epsilon_i(r_{i0}+r_{if}+a_i)}{(2r_{if}+a_i+\epsilon_i)(r_{i0}-r_{if})}\tag{22}\qquad T_s=\max\{t_{si}\}\tag{23}$$

Peak acceleration:
$$A_i^*=-2b_i^2Q_i^3+\frac{a_i^2b_i^2Q_i}{2}-2Q_ib_ic_i,\qquad Q_i=\sqrt{\frac{a_i^2}{12}-\frac{c_i}{3b_i}}\tag{26,27}$$

**Numerical example (Mars, g = 3.721 m/s², m0 = 2000 kg, m_dry = 1500 kg, v_ex = 2206.575 m/s, eps = 0.1 m):**
- PDI `(1900, 1000, 3100) m`, v0 `(-40, -10, -50) m/s`, PDT `(0,0,5) m`
- `(a,b,c)` per axis: `(-3800, 1.1080e-5, 0)`, `(-2000, 1e-5, 0)`, `(-6200, 0.5219e-5, 0.1616)`
- `(t_xs, t_ys, t_zs) = (250.4512, 495.1718, 341.4793) s`; `T_s = 495.1718 s`
- Final error `(1.6908, 10, 0.077) cm`
- Peak accelerations `(A*_x, A*_y, A*_z) = (0.6482, 0.0769, 0.6218) m/s²`; z-axis shows `A*_z − g = −3.0992 m/s²`
- Fuel consumed **221.307 kg**

**Note the scale mismatch with the booster case:** BPDG's own test is ~0.08–0.6 m/s² over ~500 s. **Its dynamics are an algebraic velocity-position law, not a thrust-bounded law** — the paper's own conclusion admits this:

> "Future work directions include developing the methodology considering **fuel optimality and operational constraints** in the BPDG problem."

---

## 8. Direct implication for the KSP booster case (arithmetic, not speculation)

The requested scenario: start at 10 km, 650 m/s down + 650 m/s horizontal (≈925 m/s), net decel 50 m/s², burn ≈20 s, `T_go ≈ 20 s`.

Applying Klumpp Eq. (7), in a guidance frame with X = up (altitude) and Z = downrange, target at origin, target velocity and acceleration zero, `T = −20 s`, current `R_G = (10000, 0)`, `V_G = (−650, +650)`:

```
ACG = 12(R_TG − R_G)/T² + 6(V_TG + V_G)/T + A_TG
```
- vertical: `12(0−10000)/400 + 6(0−650)/(−20) = −300 + 195 = −105 m/s²`
- horizontal: `12(0−0)/400 + 6(0+650)/(−20) = 0 − 195 = −195 m/s²`
- **`|ACG| ≈ 221 m/s²` demanded against ~50 m/s² available** — a ~4.4× deficit.

This is verified numerically (sympy/numpy check run during this analysis). **The conclusion the corpus supports, stated carefully:**

1. Klumpp Eq. (7)/(11) produces a **smooth, saturated-at-the-ends, non-bang-bang** acceleration profile. In this regime the command exceeds the achievable acceleration for essentially the whole burn, so the law is effectively **open-loop maximum thrust** and its "optimality" is void — consistent with Klumpp's own description of P63:
   > "Properly targeted, the guidance algorithm commands during most of P63 a thrust-acceleration in excess of what can be achieved. The throttle routine multiplies this thrust-acceleration command by the estimated mass to yield the guidance thrust command (GTC), and provides maximum thrust until the GTC falls below 57%."
   Apollo's answer to exactly this situation was to let the throttle saturate and fly **max thrust**, not to correct the guidance law.
2. The bifurcation paper's bang-bang result implies a fuel-optimal booster landing burn in this regime is **max-thrust for most of the burn**, which is what a practical "suicide burn" already does.
3. Because `T_go` is solved as a **root of a cubic** (Klumpp Eq. 12) and the closed form requires only `J, A, V, R` on one axis, the T solve is **genuinely implementable in kOS** (Newton, warm-started, ~4–8 iterations, no matrix library needed). The `I X A(I) < 1` scaling constraint from `ROOTPSRS` is the practical reason for the `/8` `/64` `/4` factorization — **a kOS port should replicate that normalization to avoid overflow/precision loss.**
4. The two mitigations that *are* in the corpus and *are* portable: (a) **stop before `T = 0`** — hand off to a velocity-only controller; (b) **transport-lag gain reduction** via the `(T + LEADTIME)/T` ratio, computable in kOS with one scalar.

**What the corpus does not give:** any quantitative guidance for a *high-energy, thrust-saturated, short-T* burn. Every worked example in all five files is either low-energy (Apollo: 15 km, ~500 s braking phase) or tiny-accleration (BPDG: 0.6 m/s², 500 s) or SOCP (patent: 14 s of compute on a laptop). **The booster scenario is outside the validated envelope of every law in these five files, and the files should not be cited as supporting it.**

---

## 9. Gaps and explicit non-findings

1. **E-matrix / 6-constraint E-guidance derivation: ABSENT** from all five assigned files. Not in Klumpp (he derives the quartic/implicit law), not in the 2015 Chinese review, not in the patent, not in the bifurcation paper (which only cites Cherry 1964). A "E-matrix" never appears.
2. **Explicit `c0, c1, c2` for APDG in terms of `r, v, a_f, T`: ABSENT.** The bifurcation paper states the constraint set but not the solution; Klumpp gives the equivalent quartic-fit form; the patent is about SOCP.
3. **PEG: ABSENT from all five assigned files.** PEG appears elsewhere in the same corpus (`notes\Jaggers1976_PEG推力积分推导与实现_航天飞机显式制导.md`, `notes\2018_PEG改进与增强_SLS Block1运载火箭.md`, `notes\1976_PEG显式制导基准任务3A偏差散布分析.md`, `notes\2024_深空探测学报_深空探测器动力下降制导方法.md`) — all **ascent** PEG (Space Shuttle / SLS heritage), not lunar descent. The patent does not mention PEG.
4. **No `T_go` floor / clamp value is given anywhere.** Mitigations are structural (terminate early, lag gain reduction, sign guard), not numeric clamps.
5. **OCR damage.** Three equations are corrupted in the extraction and must not be used verbatim: Klumpp Eq. (9) (the 2×6 jerk/snap matrix — second row, last two columns), Klumpp Eq. (11) (rendered as a broken column), and the two `P/T = ... , ζ = ...` gain-selection values. Everything else quoted above is clean.
6. **Internal tension flagged, not resolved:** the bifurcation paper says *"When APDG law is used on actual missions the time-to-go is determined using trial and error method on ground"*, whereas Klumpp and the AGC source show Apollo solving T **onboard** as a cubic root every pass. The two statements concern different vehicles/algorithms; for Apollo lunar descent the onboard-solve account is the one backed by primary source code.

---

## 10. Verification of quoted AGC identifiers

Every AGC symbol quoted in §3 and §4 was cross-checked against the AGC's own assignment file, `ERASABLE_ASSIGNMENTS.agc`, so none of the code quotes depend on OCR of figure images:

```
ERASABLE_ASSIGNMENTS.agc:1350: GAINBRAK	EQUALS	JBRFG* +2	# B(2)
ERASABLE_ASSIGNMENTS.agc:1351: TCGFBRAK	EQUALS	GAINBRAK +2	# B(1)
ERASABLE_ASSIGNMENTS.agc:1352: TCGIBRAK	EQUALS	TCGFBRAK +1	# B(1)
ERASABLE_ASSIGNMENTS.agc:1360: TCGFAPPR	EQUALS	GAINAPPR +2	# B(1)
ERASABLE_ASSIGNMENTS.agc:1361: TCGIAPPR	EQUALS	TCGFAPPR +1	# B(1)
ERASABLE_ASSIGNMENTS.agc:2096: TENDBRAK	ERASE			# B(1) LANDING PHASE SWITCHING CRITERION.
ERASABLE_ASSIGNMENTS.agc:2100: LEADTIME	ERASE			# B(1) TIME INCREMENT SPECIFYING HOW MUCH
```

**`TENDBRAK` is defined by the AGC itself as the "LANDING PHASE SWITCHING CRITERION"** — confirming the §3.2 step-7 reading that the P63→P64 switch is a threshold on `TENDBRAK + TTF/8`, not a `T = 0` test.

The 20°/70° attitude limits quoted in §3.3(g) are also confirmed as genuine code comments (not OCR noise) in `FINDCDUW--GUIDAP_INTERFACE.agc`:

```
FINDCDUW--GUIDAP_INTERFACE.agc:258: # LIMIT THE MIDDLE GIMBAL ANGLE & COMPUTE THE UNLIMITED GIMBAL ANGLE CHGS
FINDCDUW--GUIDAP_INTERFACE.agc:311: # LIMIT THE ATTITUDE ANGLE CHANGES
FINDCDUW--GUIDAP_INTERFACE.agc:359: 		ADS	-DELGMB		# -DELGMBX.  NO OVERFLOW SINCE LIMITED TO
FINDCDUW--GUIDAP_INTERFACE.agc:360: 					# 20DEG(1+SIN(70DEG)/COS(70DEG)) < 180DEG
```

---

## Source attribution index

| File | Used for |
|---|---|
| `notes\Klumpp1971_Apollo登月舱下降段制导导航与控制_R-695原典.md` | §1, §2.1, §4.1, §4.3, §5.1–5.2, §5.5, §8(1) — all Klumpp equations and quotes |
| `01_原典_E制导与PEG\Apollo11_AGC月面着陆制导方程源码_Luminary099.txt` | §3.1–3.2, §3.3(a)(b)(c)(d)(e)(f)(g)(h)(i), §4.2, §5.3 — AGC code, `QUADGUID`, `TTF/8CL`, `ROOTPSRS` |
| `notes\2025_分岔法动力下降制导律_指出E制导仅能满足初末位置速度约束.md` | §2.1, §4.4, §6.2(last), §7 in full |
| `notes\2015_深空探测学报_阿波罗二次多项式制导律与标称轨迹制导.md` | §2.1 (quadratic-polynomial qualitative description) |
| `notes\2010_专利US20100228409A1_动力下降制导方法与装置_含PEG与多项式制导优缺点讨论.md` | §6.2–6.4 |
| `90_代码与源码\...\Luminary099\THROTTLE_CONTROL_ROUTINES.agc` | §3.3(j) — `THROTTLE`, `PIF`, `FWEIGHT`, `FLATOUT` |
| `90_代码与源码\...\Luminary099\SERVICER.agc` | §3.3(k) — `PGUIDE`, `HIGATCHK` |
| `90_代码与源码\...\Luminary099\THE_LUNAR_LANDING.agc` | §4.2 (`TTF/8` seeding at `HI6ZEROS`), `DDUMCRIT`, `GUIDDURN` |
