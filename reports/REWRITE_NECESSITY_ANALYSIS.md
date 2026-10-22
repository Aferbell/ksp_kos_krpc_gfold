# REWRITE NECESSITY ANALYSIS — `dev/krpc/gfold_land.py` (6076 lines)

> Scope: determine what is TRULY NECESSARY in the control flow before a rewrite.
> Method: every claim below was verified by reading `dev/krpc/gfold_land.py` at the cited
> line numbers, and — where the claim is about *behaviour* — by replaying the recorded
> flight logs in `log-B1040-9-gfold/`.
> **No flight code was modified. This report is the only file written.**
>
> Evidence logs used (all replayed):
> | log | rows | note |
> |---|---|---|
> | `gfold_log_20261007_212924.csv` | 123 | most recent; landed alt=16.64 |
> | `gfold_log_20261007_204325.csv` | 91 | |
> | `gfold_log_20261007_152048.csv` | 89 | the "attitude loss" flight of `ATTITUDE_ROOT_CAUSE_20261007.md` |
> | `gfold_log_20261003_153758.csv` | 88 | older schema (no `ca_*`) |
>
> Discrepancies found against the prior reports are flagged inline as **CORRECTION**.

---

## 0. Executive summary

1. The **irreducible flight-critical core** is small: `state()` -> sample the plan ->
   one horizontal acceleration law -> one tilt state with cap/slew -> one vertical
   magnitude -> `apply()`'s throttle + attitude PID. Everything else is either
   diagnostics or a switch that is pinned.
2. **`TILT_FIRST_ENABLE = True` (line 551) makes the entire old-law attitude path dead.**
   `target_a_` (2858) and `limit_tilt_dir(...)` at 3103 have **no effect on any
   actuator** — verified: `tilt_cmd == tf_cmd` in 123/123, 91/91, 89/89, 88/88 frames.
3. **The vertical channel is structurally polluted by a round trip.** The commanded
   vertical acceleration is `|conic_clamp(target_a)| * cos(tilt_cmd)`
   (3563/3565), not a vertical law. In the latest flight the old-law PD vertical term
   is *negative* in **72/123 = 58.5%** of frames (median **-8.10 m/s^2**), and only
   `conic_clamp`'s lower bound keeps it positive (clamp output min = 5.002, median 9.95).
   **The clamp is load-bearing; the PD that feeds it is not.**
4. **The `AVER_*` family (816-898, 3563-3627) never changes a value.** `av_up == av_old`
   in **0 differing frames out of 391** across four flights (the brief said ~4%;
   **CORRECTION: it is 0.0%**).
5. The **tilt angle is limited by two different, unrelated caps**: `tilt_now`
   (3002, the solver's 85 deg -> 5 deg thrust cone, fed to `conic_clamp`) and `_tcap`
   (3420, `clamp(alt/3, 5, 35)`, fed to the tilt law). At handoff the first is **85 deg**
   and the second is 35 deg. Only the second is actually a design constraint.
6. The file's real hazard is not the switches; it is that **a single physical quantity
   is carried by up to four variables that are allowed to drift apart** (Section 4).

---

## 1. The minimal necessary control flow

### 1.1 Call graph actually executed

```
main() 6017
  GfoldLander.run() 5114
      gate wait loop                     5114-5215   ESSENTIAL
      first solve (non-blocking)         5217-5270   ESSENTIAL
      main control loop                  5290-5789   ESSENTIAL
          state()                        2212-2275   ESSENTIAL
          [replan]                       5504-5514   OPTIONAL (log shows 1)
          nav_mode == 'hold'             5521-5646   ESSENTIAL (first 1-4 frames)
          track()                        2701-3777   ESSENTIAL
          apply()                        4339-4769   ESSENTIAL
          draw()                         5859-5898   DIAGNOSTIC
          log.row(...)                   5746-5771   DIAGNOSTIC
```

Supporting, called from the above: `target_frame()` 2310-2372,
`body_rotation_speed()` 2277-2308, `a_cap_real()` 4002-4064, `update_thrust_scale()`
3803-3865, `_predict_est_time()` 2411-2438, `hold_accel()` 2440-2462,
`vessel_profile1()` 2374-2409, `solve()` 2464-2552, `_start_resolve()`/`_collect_resolve()`
5060-5113, `find_nearest_index()` 2554-2604, `sample_index()` 2606-2674,
`_lead_index()` 2676-2699, `limit_tilt_dir()` 1301-1328, `PID` 1358-1427,
`_limit_ref_slew()` 3867-3956, `_att_sched_factor()` 3958-4000,
`conic_clamp()` 4247-4337, `_gfold_floor()` 4066-4100,
`_gfold_floor_effective()` 4102-4219, `_solver_cone_deg()` 4221-4245.

### 1.2 The irreducible sequence (what a rewrite must contain)

| # | Step | Lines (current) | What it computes | Why irreducible |
|---|---|---|---|---|
| 1 | **Gate** | 5183-5184 | `alt < GATE_ALT and vmag < GATE_VMAX and cone_ok and alt > GATE_ALT_MIN` | Without it the script flies the wrong vehicle. |
| 2 | **State estimation** | 2212-2275 | `alt, dist_hz, vz, vh, vmag, mass, tp, tv` in the target hybrid frame (`x`=zenith, `y`=north, `z`=east) built at 2310-2372 | This frame is the *only* reason `x_i - error` at 2857/3184 is dimensionally valid (comment 2724-2727). Getting `velocity=` wrong here caused the 2026-09-26 failure (2340-2368). **Safety-critical.** |
| 3 | **Outer-loop prediction** | 2374-2409, 2411-2438, 2440-2462 | `x0_state` = 2nd-order extrapolation of `(tp,tv)` by `est_time` | The solver takes 0.15-3.9 s; without extrapolation the plan starts up to 800 m away from the vehicle (2383-2395). |
| 4 | **Plan sampling: index** | 2740-2798 | `gfold_n_i = clamp(max(n_i - dt*0.2*N/tf, find_nearest_index(...)), 0, N-2)` | `find_nearest_index` (2554-2604) projects the vehicle onto the plan; monotonicity (2751) and the `N-2` cap (2798) prevent the two known freeze/hover failures. **Safety-critical.** |
| 5 | **Plan sampling: interpolation** | 2826, 2839-2844 | `(x_i, v_i, u_i) = sample_index(n_i)`, plus a lead sample `(x_lk, v_lk, u_lk)` at `_lead_index` | `x_i,v_i,u_i` are the plan reference; the lead sample is the 1.5 s look-ahead that cancels the measured 1.2-2.2 s airframe lag (2828-2838). |
| 6 | **Guidance law (horizontal)** | 3289-3301 | `T_h = clamp(2|dp|/|dve|, 6, 8)`; `kv = 4/T_h`; `kp = 6/T_h^2`; `_ah_vec = u_h + kv*dve + kp*dp` | This is the only horizontal law that is live. `zeta = 0.816` is T-invariant by construction (582-594). |
| 7 | **Guidance law (tilt magnitude)** | 3336, 3399-3402 | `_tilt_plan + _tcorr` -> `_tilt_want`; with `TILT_VEC_MODE` the effective form is `_tilt_want = _tcorr = atan2(|_ah_vec|, |_p0|)` | Converts the horizontal acceleration demand into a tilt demand. |
| 8 | **Tilt limit** | 3418-3426 | `_tcap = clamp(alt/TILT_CAP_SLOPE, 5, 35)`; `_tilt_want = clamp(_tilt_want, 0, _tcap)` | Prevents the airframe from going past the angle at which vertical thrust collapses (489-497). **Safety-critical.** |
| 9 | **Terminal taper** | 3437-3456 | below `TILT_TERM_ALT=90 m`, shrink tilt but hold `>= _tcap` floor | Guarantees a vertical touchdown (541-544). **Safety-critical.** |
| 10 | **Tilt rate limit** | 3458-3485 | `_tilt_cmd = _prev_tilt + clamp(_tilt_want - _prev_tilt, +-slew*dt)`; `slew = TILT_SLEW_LAG_DEG * sqrt(alpha*kp)` | Verified live: **34/123 frames sit exactly on the rate limit**, and it is the physical expression of "can start and can stop". **Safety-critical.** |
| 11 | **Azimuth** | 3486-3537 | `_az = _ah_vec/|_ah_vec|`; hold last valid when `|_dp| < 1.5 m` | `atan2` of a near-zero horizontal vector is ill-conditioned (`plan |u_h|` p50 = 0.70 vs vertical 9.90). |
| 12 | **Vertical magnitude** | 3563-3612 | `_amag = |target_a|`; `_a_up = _amag*cos(tilt)`; then optional need-clamp (usually inactive); `_a_h = _a_up*tan(tilt)` | **This is the round trip** (Section 4.1). It is what makes the vehicle descend. |
| 13 | **Attitude target rebuild** | 3638-3643 | `_dir_new = [cos(tilt), sin(tilt)*az_n, sin(tilt)*az_e]` normalised | Constructed directly from tilt so that `tilt(target_a) == tilt(target_direction)` exactly (3629-3637). |
| 14 | **Command vector + reference slew** | 3648-3651, 3692, 3694 | `a_cmd_vec = [a_up, a_h*az]`; `target_a = a_cmd_vec`; `target_direction = _limit_ref_slew(target_direction)` | Feeds `apply()`. `_limit_ref_slew` verified live: `ref_clip=1` in 21/123 frames. |
| 15 | **Thrust magnitude (actuation)** | 4370-4375 | `throttle = clamp(|target_a| / a_cap_real(), 0, 1)` | Simple, correct. |
| 16 | **Thrust authority** | 4002-4064 | `a_cap = available_thrust * _thrust_scale() / mass` | The 1.414 calibration is empirically required on this kRPC build; without it the throttle floor exceeds true hover and the rocket climbs (4025-4062). **Safety-critical.** |
| 17 | **Attitude command (actuation)** | 4396-4626 | body-frame error via `_angle_around_axis`, PID with `rate=avel_local[0 or 2]*RATE_D_GAIN + ff`, `control.pitch/yaw/roll` | The only path to the gimbal. |
| 18 | **Termination** | 5780-5788 | `alt <= TARGET_ALT+1` or `situation == 'landed'`; abort if `vz > 5` | Prevents the 215838 climb-away (815-834). |

Everything in `track()` outside steps 4-14 above, and everything in `apply()` outside
steps 15-18, is either diagnostic or an inactive branch (Section 3).

---

## 2. The two parallel law paths

`TILT_FIRST_ENABLE = True` at line 551. Verified live at runtime: log column `tf_on = 1.0`
in 123/123 frames; `tf_err` is empty in all four logs (the exception fallback at 3663-3676
never fired).

### 2.1 Live vs dead, statement by statement

| Line | Statement | Status when `TILT_FIRST_ENABLE=True` |
|---|---|---|
| 2857 | `target_a = u_i + (v_i-vel)*K_VEL + (x_i-error)*K_POS` | **LIVE, indirectly.** Only consumer is `_amag = |target_a|` at 3563. Nothing else reads the vector. |
| 2858 | `target_a_ = u_i_ + (v_i_-vel)*K_VEL + (x_i-error)*K_POS` | **DEAD.** Only consumer is 3095/3093. |
| 2852 | `_x_i_, v_i_, u_i_ = sample_index(n_i + step)` | **DEAD** as to `_x_i_` (never read again) and as to `v_i_`/`u_i_` (only feed 2858). |
| 2946-2968 | `a_cap`, `min_mag` via `_gfold_floor_effective` | **LIVE** (feeds `conic_clamp`). |
| 3000-3023 | `tilt_traj`, `tilt_now` from `_solver_cone_deg`, cone-mismatch warning | **LIVE** (feeds `conic_clamp` at 3059/3062), but see Section 4.10. |
| 3059 | `target_a = conic_clamp(target_a, min_mag, max_mag, tilt_now, min_mag_override=min_mag, plan_a_up=_plan_a_up)` | **LIVE** — this is the true vertical command (via 3563/3565). |
| 3062 | `target_a_ = conic_clamp(target_a_, ...)` | **DEAD.** |
| 3090-3093 | degraded `if not self.gfold_traj_ready:` sets `target_a = target_a_ = G0+u_i` | **LIVE** for `target_a` (still reaches 3563), dead for `target_a_`. |
| 3095 | `self.target_direction = target_a_ / |target_a_|` | **DEAD** — overwritten at 3643. |
| 3103 | `self.target_direction = limit_tilt_dir(self.target_direction, MAX_TILT_CMD_DEG)` | **DEAD** — overwritten at 3643. (Still live for the hold branch at 5597 and the `apply()` fallback at 4394.) |
| 3125/3152 | old `_limit_ref_slew` call site | Already **moved** to 3692 by step 18(a); 3125 is a comment. |
| 3166-3662 | the tilt-first block | **LIVE** (all of it, minus diagnostics). |
| 3692 | `self.target_direction = self._limit_ref_slew(self.target_direction)` | **LIVE** for both branches. |
| 3694 | `self.a_cmd_vec = target_a` | **LIVE** but redundant — see 4.4. |
| 3695-3697 | `self.tilt_cmd = atan2(|target_a[1:3]|, target_a[0])` | **DEAD** (`if not TILT_FIRST_ENABLE`). `tilt_cmd` was already set at 3652. |

**Verified consequence:** `tilt_cmd == tf_cmd` (the tilt-first internal value) in
**123/123, 91/91, 89/89, 88/88** frames across the four logs. The old-law computation at
3696-3697 never runs, and 3103 never affects an actuator.

### 2.2 Variables written by one path and overwritten/ignored by the other

| Variable | Old-law writer | Tilt-first writer | Winner |
|---|---|---|---|
| `self.target_direction` | 3095, 3103 | **3643**, then 3692 | tilt-first |
| `target_a` (local) | 2857, 3059, 3092 | **3651** (`= self.a_cmd_vec`) | tilt-first |
| `target_a_` (local) | 2858, 3062, 3093 | never | dead value, no winner |
| `self.tilt_cmd` | 3696 (guarded `if not`) | **3652** | tilt-first |
| `self.a_cmd_vec` | — | 3648, then 3694 | tilt-first |
| `_a_up_new`/`_a_h_new` | — | 3593-3619 | tilt-first only |
| `self.tf_tilt_prev` | — | **3485** | tilt-first only |
| `self.min_mag_eff` | 2968 (shared) | — | shared, read at 4386 |
| `_tilt_cmd` | — | 3480-3484 | tilt-first only |

The two paths are **not symmetric**: the old path still *produces* `target_a`, and the new
path *consumes* it (3563). That is the single most important structural fact in the file.

### 2.3 What the old law still contributes

Replaying `gfold_log_20261007_212924.csv` and recomputing the pre-clamp vertical PD term
`u_i[0] + 0.8*(plan_vz - vz) + 0.5*(x_i[0]-error[0])`:

```
 raw PD vertical :  min = -19.73   p50 =  -8.10   p90 =  23.18   max =  47.57
 conic_clamp out
   (= av_old)    :  min =   5.00   p50 =   9.95   p90 =  23.17   max =  32.44
 |raw - clamped| :  p50 = 17.27   p90 = 26.77   max = 29.63
 frames where the clamp CHANGED the vertical value : 123/123 = 100%
 frames where raw PD vertical < 0                   :  72/123 =  58.5%
```

So the old law's PD **is** doing something, but what survives to the actuator is dominated
by `conic_clamp`'s lower bound, not by the PD. `av_up / av_plan` is
p50 = 0.92, p90 = 1.72, max = 2.80 — the vertical command bears little relation to the plan.

---

## 3. Genuinely necessary vs removable

### 3.1 `conic_clamp` (4247-4337) — **ESSENTIAL** (safety-critical)

Evidence: 100% of frames have their vertical value changed by it; the pre-clamp value is
negative in 58.5% of frames of the latest flight (and 17/91, 4/89 in the two earlier ones).
Its `min_mag`-derived floor is what keeps the vehicle descending instead of falling.

Which part does the work — replaying the exact expression
`a_ver_min = max(cos(cone)*min_mag, max(0, min(AVER_FLOOR, plan_a_up, G0*0.99)))`:

```
 base-branch frames (err_up<=0 and net_up>=0) :  14/123  -> min_mag = 9.41 m/s^2
 release-branch frames                        : 109/123  -> min_mag = min(0.05*a_cap, 1.02*|u_i|)
 frames where cos(cone)*min_mag > AVER_FLOOR  :  14/123  (values 6.13 .. 9.41)
 frames where AVER_FLOOR is the binding floor :   1/123  (av_old == 5.0 exactly)
```

=> the **`min_mag` path (via `_gfold_floor_effective`) is the live one**; `AVER_FLOOR`
binds in ~1% of frames. The **horizontal branch** (4336, `a_hor = min(a_hor, a_ver*tan(max_tilt))`)
is redundant with 3612 (`_a_h_new = _a_up_new * tan(_tr)`) once the tilt-first law owns
the tilt. Classify the whole function **ESSENTIAL**, the `AVER_FLOOR` term **DEAD**,
the `a_hor` branch **ESSENTIAL-BUT-COULD-BE-SIMPLER**.

### 3.2 `_gfold_floor` (4066-4100) / `_gfold_floor_effective` (4102-4219) — **ESSENTIAL**

Safety-critical: it is the bound that prevents the "zero-throttle trap" documented at
4155-4173 (36/70 frames at `thr_cmd == 0.000` for 7.2 s, free-fall from 597 m to 305 m).
Verified active: 109/123 frames take the release branch, 14/123 the base branch.
`_gfold_floor` is **ESSENTIAL-BUT-COULD-BE-SIMPLER** — the three-way
`min(GFOLD_THROTTLE_MIN, GFOLD_THROTTLE_MARGIN*hover)`, then
`min(GFOLD_THROTTLE_RELEASE*a_cap, 1.02*|u_i|)` collapses to one expression.

### 3.3 `_limit_ref_slew` (3867-3956) — **ESSENTIAL** (and it is *not* dead)

**CORRECTION to `ATTITUDE_ROOT_CAUSE_20261007.md` §3.1**, which concluded the limiter is
dead code because 3125 was overwritten by 3616. That was true when the report was written;
step 18(a) (comment at 3126-3152, active call at **3692**) *moved* the call after the
`TILT_FIRST_ENABLE` block, so it now acts on the final direction.
Verified in the current logs: `ref_clip = 1` in **21/123** frames of 212924 and **12/91** of
204325 — i.e. it is genuinely limiting, and the report's "97.8% sticky" reading is also
fixed (`ref_clip` is reset to 0 at 3914 every frame).

Behaviour is still asymmetric: it limits the attitude *direction* but nothing limits the
azimuth rate separately. Essential, but **ESSENTIAL-BUT-COULD-BE-SIMPLER**.

### 3.4 `_att_sched_factor` (3958-4000) — **ESSENTIAL**

Verified live: `sched` p50 = 0.815, range up to 1.281, nonzero in 123/123 frames.
It multiplies `CTRL_X_ROT_KP`/`CTRL_Z_ROT_KP` at 4445-4448, which is the mechanism that
keeps `zeta` roughly constant against an 8x swing in `alpha_max` (3963-3976).
Keep, but simplify: the `20.0` reference at 3997 is a magic constant that should be derived
from `CTRL_X_ROT_KP`/`CTRL_Z_ROT_KD`.

### 3.5 The vertical clamp / floor logic at 3538-3627 — **mostly DEAD**

This is the strongest finding of this analysis.

```
 Over FOUR flights:
   frames where av_up != av_old          :  0 / 123, 0 / 91, 0 / 89, 0 / 88   -> 0.0%
   frames where av_clamp == 1            :      5,      4,      4,      5
   frames where av_need is NaN (inactive):  95.9%,  95.6%,  95.5%
   frames where av_need < av_old         :  0 / 123, 0 / 91, 0 / 89, 0 / 88
```

Because `AVER_CLAMP_ENABLE_BELOW=True` (888) and `AVER_CLAMP_H_MAX=10.0` (879), the clamp
is only armed below 10 m. In the 5 armed frames of 212924 the need values were
`29.5, 32.9, 38.9, 61.0, 76.4` against `av_old = 26.1, 24.6, 30.6, 24.0, 32.3` — the bound
is *always above* the value it is supposed to cap, so `min()` never selects it.

**Classification: the whole 3569-3602 clamp, `AVER_FF_CLAMP_ENABLE` (856),
`AVER_TGT_VZ` (860), `AVER_CLIMB_MARGIN`/`AVER_CLIMB_K` (861-862),
`AVER_USE_CLIMB_GUARD` (878), `AVER_CLAMP_H_MAX` (879), `AVER_CLAMP_ENABLE_BELOW` (888),
and log columns `av_need`/`av_clamp`/`av_hgate` are DEAD in flight.**
The climb-guard scenario they defend against (874: divergence to h=624 km) is real but is
already covered by the `vz > 5.0` abort at 5786.
`AVER_FLOOR` (382) itself binds in ~1% of frames — **DIAGNOSTIC-ONLY** in effect.

> **CORRECTION:** the brief states `av_up == av_old` except in ~4% of frames. Measured
> value is **0.0%** over 391 frames across four flights. The `av_up`/`av_old` columns are
> the same number; they are two names for one quantity (Section 4.2).

### 3.6 `AVER_*` family, itemised

| Symbol | Lines | Class |
|---|---|---|
| `AVER_FF_CLAMP_ENABLE` | 856 | **DEAD** (clamp never binds) |
| `AVER_H_FLOOR` | 857 | DEAD (only used inside the dead clamp) |
| `AVER_TGT_VZ` | 860 | DEAD |
| `AVER_CLIMB_MARGIN`, `AVER_CLIMB_K` | 861-862 | DEAD |
| `AVER_USE_CLIMB_GUARD` | 878 | DEAD |
| `AVER_CLAMP_H_MAX` | 879 | DEAD |
| `AVER_CLAMP_ENABLE_BELOW` | 888 | DEAD |
| `AVER_FLOOR_ENABLE`, `AVER_FLOOR` | 381-382 | binds ~1/123 -> **ESSENTIAL-BUT-COULD-BE-SIMPLER** (collapse into the `min_mag` floor) |

### 3.7 `TILT_*` family

| Symbol | Lines | Class | Evidence |
|---|---|---|---|
| `TILT_FIRST_ENABLE` | 551 | **DEAD switch** | pinned True; `tf_on=1.0` always |
| `TILT_RATE_MAX_DEG` | 552 | DEAD | only reachable when `TILT_SLEW_ADAPT_ENABLE=False` (3468) |
| `TILT_CORR_TGO_MODE` | 567 | DEAD switch | pinned True; else-branch 3384-3396 unreachable |
| `TILT_CORR_VEL_GAIN`, `TILT_CORR_POS_GAIN` | 568-569 | DEAD | only in the unreachable else |
| `TILT_CORR_POS_MAX_DEG` | 572 | DIAGNOSTIC-ONLY | comment 573-574 says so; used at 3342 for column only |
| `CA_T_MIN`, `CA_T_MAX` | 616-617 | **ESSENTIAL** | 3289-3290; `ca_T` p50=6 (pinned at `CA_T_MIN` in 212924!) |
| `TILT_TERM_ALT` | 618 | **ESSENTIAL** (safety) | 3437; taper active in 16/123 frames |
| `TILT_TERM_MIN_DEG` | 621 | DEAD | only 3450-3452, inside `TILT_CAP_ALT_ENABLE=False` |
| `TILT_AZ_MIN_ERR` | 629 | **ESSENTIAL** | 3510 |
| `TILT_CAP_ALT_ENABLE`, `_LO`, `_HI`, `_SLOPE` | 673-676 | **ESSENTIAL** (safety) | 3419-3422; `tf_cap` = 35 in 105/123 frames, decays to 5.54 |
| `TILT_VEC_MODE` | 681 | DEAD switch | pinned True |
| `TILT_TGO_EGUIDE` | 689 | **DIAGNOSTIC-ONLY** | `_tgo` only reaches 3357/3358/3365 |
| `TILT_VERR_MODE` | 690 | DIAGNOSTIC-ONLY | `_vhoriz` only reaches 3443/3449 (disabled branch) + 3359 |
| `TILT_RESP_TIME` | 691 | DIAGNOSTIC-ONLY | 3238 -> only 3365 |
| `TILT_TGO_MIN` | 697 | DIAGNOSTIC-ONLY | 3233/3239 -> only diagnostics |
| `TILT_GAIN_CAP_ENABLE`, `TILT_GAIN_MAX` | 706-707 | **ESSENTIAL-BUT-SIMPLER** | 3296-3297; `ca_kv` p50 = 0.667, max 1.0, cap 1.25 never binds |
| `TILT_LEAD_ENABLE`, `_SEC`, `_MAX_FRAC` | 735-739 | **ESSENTIAL** | 3171/3182/3183; `tf_lead_n` = 6.41 nodes constant |
| `TILT_RESP_FRAC` | 744 | DIAGNOSTIC-ONLY | 3238 |
| `TILT_MIN_TGO_KEEP` | 751 | **DEAD** | **never referenced anywhere in the file** |
| `MAX_TILT_CMD_DEG` | 501 | DEAD in the tilt path | only 3103 (dead), 3322/3424 (disabled branch), 4395/5597 (hold + apply fallback) |

### 3.8 `TILT_SLEW_*` family — **ESSENTIAL**

`TILT_SLEW_ADAPT_ENABLE` (800), `TILT_SLEW_LAG_DEG` (801), `TILT_SLEW_KP_REF` (802),
`TILT_SLEW_MIN_DEG` (805), `TILT_SLEW_MAX_DEG` (806) drive 3469-3481.
Verified live: `tf_slew_allow` p50 = 6.81 deg/s, max 10.7, floor 5.0;
**34/123 frames have `tilt_cmd` exactly on the rate limit**.
This is the one limiter that consistently binds and it is the direct expression of
"the airframe can start and stop a slew of this size" (768-772).
Classify **ESSENTIAL**; the five constants could collapse to two (`lag`, `kp_ref`)
plus safety clamps.

### 3.9 Other blocks

| Block | Lines | Class |
|---|---|---|
| `find_nearest_index` + monotonic + `N-2` cap | 2554-2604, 2751, 2798 | **ESSENTIAL** (safety) |
| `sample_index` arrival sentinel + `_sample_arrived_ok` | 2652-2663, 2818-2824 | **DEAD in practice** — `tf_arrived=0` in 391/391 frames because `gfold_n_i` is capped at `N-2` (2798), so `index >= N-1` is unreachable. Keep as a defensive guard only. |
| diagnostics block 13(a) | 2897-2944 | **DIAGNOSTIC-ONLY** (`vec_chk` = 0.0 constant in every frame) |
| plan-quality diagnostics 11(a) | 3710-3735 | **DIAGNOSTIC-ONLY** |
| horizontal-error direction ⑧ | 3757-3776 | **DIAGNOSTIC-ONLY** |
| `_read_actuator` | 4771-4792 | DIAGNOSTIC-ONLY |
| `_dbg_cols` | 4794-4983 | DIAGNOSTIC-ONLY (122 keys) |
| segment timers | 5298-5299, 5376-5378, 5644-5645, 5713-5714 | DIAGNOSTIC-ONLY; `seg_rpc` reported 0.0 constant |
| `draw` / `draw_target_marker` / `_ensure_lines` | 5796-5898 | DIAGNOSTIC-ONLY (`DEBUG_LINES=False`, 926) |
| frozen-game detector | 5399-5415 | **ESSENTIAL** (safety) — prevents a false takeover abort while paused |
| takeover verification | 4991-5056, 5421-5425 | **ESSENTIAL** (safety) — the kOS/kRPC contention guard |
| `hold` branch | 5521-5646 | **ESSENTIAL** — verifies `thr`/`dir` are written before the plan exists; without it there is an uncontrolled window (5231-5250) |
| `final` branch | — | **already deleted** (5648-5674). `FINAL_RADIUS`/`FINAL_HEIGHT`/`FINAL_THROTTLE`/`FINAL_KP` (1076-1079) and `FINAL_THROTTLE_MIN` (1103) are **DEAD constants**. |
| terminal-guide constants | 1263-1271 (`TERM_H`, `TERM_V`, `TERM_K`, `TERM_KP`, `TERM_KD`, `TERM_AMAX`, `TERM_TILT`, `TILT_SPAN`, `EG_TMIN`) | **DEAD** — not referenced outside their definitions |
| `CONIC_TILT_DEG` | 959 | **DEAD** — 4302 uses it only when `tilt_deg is None`, and all call sites (3059, 3062) pass `tilt_now` |
| `KP_X`, `KV_V` | 946-947 | **DEAD aliases** |
| `_slew_prev_dir`, `_slew_alpha`, `_slew_active`, `_slew_wmax`, `_slew_err` | 2004-2008 | **DEAD fields** — the whole `_slew_limit` mechanism was removed (4403-4412) and the columns deleted (4984-4989) |
| `self.tilt_cap` | 1747, 3702 | **DEAD** — written, never read, no log column |
| `dbg_idx_floor` | 2801 | **DEAD** — written, never read, no column |
| `dbg_a_ver_raw` | 3065 | **DEAD** — written, never read, no column |
| `_x_i_` | 2852 | **DEAD** |

**Log-column dead weight** (measured on 212924, 149 columns):
always-empty: `thr_act`, `cone_bad`, `tf_err`, `note` (4).
constant for the whole flight: `replan_n`, `solve_ms`, `tf`, `pid_ix`, `cone_mis`, `vdesc`,
`pid_sat_x`, `pid_sat_run`, `seg_rpc`, `aver_floor`, `plan_pk_tilt`, `plan_x0_up`,
`plan_tf_used`, `vec_chk`, `tf_on`, `tf_arrived`, `ca_pc`, `draw_on` (18).
=> **22 of 149 columns carry no per-frame information.**

---

## 4. Tangled-variable inventory

Each entry: the variables, the lines, and whether an equality invariant holds.

### 4.1 `_amag` / `_a_up_old` vs the vertical command — **invariant CAN be violated (by design)**

- `_amag = |target_a|` — 3563
- `_a_up_old = _amag * cos(_tilt_cmd)` — 3565
- `_a_up_new = min(_a_up_old, _a_ver_need)` — 3593 / `= _a_up_old` — 3606
- `_a_h_new = _a_up_new * tan(_tilt_cmd)` — 3612
- `a_cmd_vec = [_a_up_new, _a_h_new*az]` — 3648-3650

`target_a` at 3563 is the output of `conic_clamp` (3059) and therefore already contains the
plan's thrust direction. Projecting it onto `cos(tilt_cmd)` and then re-deriving
`a_h = a_up*tan(tilt)` means the horizontal command is computed **twice** through two
different parameterisations. Measured consequence: `av_up/av_plan` p50 = 0.92,
**p90 = 1.72, max = 2.80**. There is no invariant; the two representations are free to
disagree and they do.

### 4.2 `av_up` vs `av_old` — **invariant HOLDS but is vacuous**

- `self.dbg_av_old = _a_up_old` — 3624
- `self.dbg_av_up = _a_up_new` — 3625

Measured: **0 differing frames in 391**. The clamp at 3569-3602 never fires, so the two
columns are the same number. A reader of the log cannot tell that the clamp is dead —
this is exactly the "column that lies" failure mode the file's own comments warn about
(3894-3912). Two variables for one quantity, with a 40-line clamp between them.

### 4.3 `tilt_cmd` vs `tf_tilt_prev` vs `tf_cmd` — **invariant HOLDS**

- `_prev_tilt = self.tf_tilt_prev` — 3461
- `self.tf_tilt_prev = _tilt_cmd` — 3485
- `self.tilt_cmd = _tilt_cmd` — 3652
- log `tf_cmd` = `dbg_tf_tilt_cmd` — 4925, set at 3658

Measured `tilt_cmd - tf_cmd` = 0 in 391/391 frames. Three names, one number. The state
(`tf_tilt_prev`) and the report (`tilt_cmd`) are separate variables that must be kept in
sync by hand at 3485/3652 — and note `_tilt_cmd` is re-clamped at 3484 *after* being rate
limited, so `tf_tilt_prev` stores the clamped value while the rate limit next frame assumes
it is the raw integrated value. Harmless today, fragile.

### 4.4 `target_a` vs `a_cmd_vec` — **invariant HOLDS, assignments are redundant**

- `self.a_cmd_vec = np.array([_a_up_new, _a_h_new*az_n, _a_h_new*az_e])` — 3648
- `target_a = self.a_cmd_vec` — 3651
- `self.a_cmd_vec = target_a` — 3694

3651 and 3694 are the identity. Two-way copying of the same object makes it impossible to
tell which one `apply()` is supposed to trust. `apply()` takes `target_a` as a parameter
(5677 -> 5697), so `self.a_cmd_vec` is used **nowhere** except being reassigned — it is a
write-only field on the live path.

### 4.5 `tilt_cmd` vs `tilt_dir_cmd` — **invariant CAN be violated** (real, measured)

- `self.tilt_cmd` = tilt of the **throttle** vector — 3652, from `_tilt_cmd`
- `self.tilt_dir_cmd` = `acos(target_direction[0])` in `apply()` — 4754-4758, i.e. the
  tilt of the **attitude** target *after* `_limit_ref_slew` at 3692

Measured `tilt_cmd - tilt_dir_cmd`: differs by more than 0.05 deg in **21/123** frames of
212924 (**max 14.89 deg**) and 8/91 of 204325 (max 4.55 deg). The file's own comment at
3698-3701 admits these are "different vectors" — this is the documented consequence.
Two physical quantities, correctly separated, but the *name* collision invites the exact
error the comment at 4631-4633 describes.

### 4.6 `tilt_now` vs `_tcap` — **invariant CAN be violated, and this one is dangerous**

- `tilt_now = _solver_cone_deg(gfold_n_i, N)` clamped to `[PCS_END_DEG, 89]` — 3002-3003,
  consumed by `conic_clamp` at 3059/3062
- `_tcap = clamp(alt/3, TILT_CAP_LO, TILT_CAP_HI)` — 3419-3422, consumed by the tilt law
  at 3425/3484

These are two unrelated caps on the same physical angle. `tilt_now` starts at
`PCS_START_DEG = 85 deg` (215) and decays to 5 deg; `_tcap` starts at 35 deg. At node 0,
`cos(85 deg) = 0.087`, so `conic_clamp`'s *tilt* constraint is effectively absent exactly
where it would matter most. The file attributes an entire flight failure to this
(2976-2999) and fixed the *floor* (25 deg) but not the *ceiling* semantics.
**A rewrite should have exactly one tilt cap.**

### 4.7 `_perr` / `_verr` (lead) vs `trk_pos_h` / `trk_vel` (not lead) — **invariant CAN be violated**

- `_dp = (x_ref - error)[1:3]` with `_x_ref = _x_lk` — 3182, 3184
- `_perr = |_dp|` — 3188; `_verr = |_dve|` — 3189
- `self.dbg_trk_pos_h = |(x_i - error)[1:3]|` — 3740 (uses `x_i`, the **non**-lead sample at 2826/3705)

Verified `tf_vhoriz == tf_verr` in 123/123 frames (they are the same number, 3207).
So the *live* law uses the lead reference while the *diagnostic* uses the current
reference. Any offline analysis that correlates `tf_perr` with `trk_pos_h` is comparing two
different physical quantities. `tf_lead_n` = 6.41 nodes = 1.5 s of plan, which at
plan speed ~60 m/s is up to **90 m** of reference displacement.

### 4.8 `_tgo`, `_tgo_eff`, `_derate`, `_vhoriz`, `_tcorr_tgo`, `_tcorr_pos` — **live computation, dead consumers**

Computed at 3200-3239 and 3340-3343. Every consumer is a diagnostic assignment:

- `_tgo` -> only 3357 (`dbg_tf_tgo`)
- `_tgo_eff` -> only 3358 (`dbg_tf_tgo_eff`)
- `_derate` -> only 3365 (`dbg_tf_derate`)
- `_tcorr_tgo` -> only 3355
- `_tcorr_pos` -> only 3356
- `_vhoriz` -> 3443/3449, inside `if not TILT_CAP_ALT_ENABLE` (disabled) + 3359

The comment at 3277-3281 says the old form `_inv_tgo = min(1.25, 1/_tgo_eff)` was
*replaced* by candidate A. It was not deleted — 60 lines of eguidance `tgo` computation now
feed nothing but log columns. **~60 lines removable with zero behavioural change.**

### 4.9 `_tilt_plan` vs `_tcorr` — **invariant violated: one of them is discarded**

- `_tilt_plan` from the lead sample — 3171-3176
- `_tcorr = atan2(|_ah_vec|, |_p0|)` — 3336
- `_tilt_want = _tcorr if (TILT_VEC_MODE and TILT_CORR_TGO_MODE) else _tilt_plan + _tcorr` — 3399-3402

Both switches are pinned True, so the live form is `_tilt_want = _tcorr` and `_tilt_plan`
is **DIAGNOSTIC-ONLY** (3655). Note the old path added the plan term and the new path does
not (the plan's `u_h` is already inside `_ah_vec`) — correct, but only by the comment at
3397-3398. A rewrite must not reintroduce the double count.

### 4.10 `raw PD vertical` vs `av_old` — **invariant violated by 17 m/s^2 median**

Section 2.3. The quantity a reader would call "the vertical acceleration command" is
`av_old`, which is `|conic_clamp(target_a)|*cos(tilt)`, not
`u_i[0] + K_VEL*(v_i-v_z) + K_POS*(x_i-z)`. They differ by p50 = 17.27, p90 = 26.77,
max = 29.63 m/s^2. In 58.5% of frames the PD value is negative.

### 4.11 Angle vs vector for the same attitude — `tilt_cmd`/`_az` vs `target_direction`

- `_tilt_cmd` (scalar) — 3480-3484; `_az` (unit vector) — 3502-3537
- `_dir_new = [cos(_tr), sin(_tr)*_az[0], sin(_tr)*_az[1]]` — 3638-3640
- `target_direction = _dir_new/_dn` — 3643, then rate-limited — 3692

Invariant before 3692: `acos(_dir_new[0]) == _tilt_cmd`. Invariant **after** 3692:
`acos(target_direction[0]) <= _tilt_cmd` — violated by up to 14.89 deg (Section 4.5).
The scalar and the vector are two representations of one attitude that the limiter is
allowed to desynchronise.

### 4.12 `min_mag_eff` vs the throttle floor actually applied

- `self.min_mag_eff = float(min_mag)` — 2968
- `self.thr_floor_last = min_mag_eff / a_cap` — 4385-4386
- log `atq_floor` derived from `dbg_floor_lift`/`dbg_a_ver_out` — 4685-4687

Measured: `throttle == floor` in only **1/123** frames, but 109/123 frames take the
*release* branch. The floor is therefore almost never the binding constraint, yet the
diagnostic is built as if it were. Consistent, but the diagnostic answers a question that
is no longer the interesting one.

---

## 5. Proposed rewrite skeleton

Design rule: **one variable per physical quantity**. State is explicit and small.
No switches. No diagnostic fields inside the controller.

```python
# ---------------- constants (vehicle-specific, keep as parameters) -------------
G0            = 9.80665
A_CAP_SCALE   = None            # calibrated online, NOT a constant
TILT_CAP_LO, TILT_CAP_HI, TILT_CAP_SLOPE = 5.0, 35.0, 3.0   # deg, deg, m/deg
TILT_TERM_ALT = 90.0                                        # m
LEAD_SEC      = 1.5                                         # s
T_GAIN_MIN, T_GAIN_MAX = 6.0, 8.0                           # s   (= CA_T_MIN/MAX)
SLEW_LAG_DEG, SLEW_KP_REF = 10.0, 2.0
FLOOR_RELEASE_K = 0.05          # of a_cap, when the plan wants to descend
THROTTLE_MARGIN = 0.96          # of hover


class GuidanceState:
    """Everything the controller remembers between frames. 6 numbers."""
    n_i        = -100.0   # arc-length index along the plan
    tilt       = 0.0      # deg, the tilt STATE (the only tilt variable)
    az         = None     # unit 2-vector (north, east); held when ill-conditioned
    dir_prev   = None     # for the reference rate limiter
    t_scale    = 1.0      # available_thrust calibration
    tilt_act_p = float('nan')


def guidance(plan, tp, tv, mass, cap, st, dt):
    """One frame of guidance. Returns the acceleration command. No diagnostics."""
    x, u, N, tf = plan['x'], plan['u'], plan['x'].shape[1], plan['tf']
    error, vel = np.asarray(tp, float), np.asarray(tv, float)

    # ---- 1. sample the plan (index mandatory-monotonic, capped at N-2) -------
    raw   = find_nearest_index(x, error, vel, tf, N)           # geometric, never time-based
    st.n_i = min(max(st.n_i, raw, 0.0), float(N) - 2.0)
    x_i, v_i, u_i = sample_index(x, u, st.n_i, N)

    # ---- 2. lead sample: cancels the measured 1.2-2.2 s airframe lag --------
    n_lk = min(st.n_i + LEAD_SEC * (N - 1) / max(tf, 1e-6),
               0.30 * (N - 1), N - 2.0)
    x_r, v_r, u_r = sample_index(x, u, max(st.n_i, n_lk), N)

    dp  = (x_r - error)[1:3]        # horizontal position error  (plan - vehicle)
    dve = (v_r - vel )[1:3]         # horizontal velocity error
    perr, verr = np.linalg.norm(dp), np.linalg.norm(dve)

    # ---- 3. THE guidance law: horizontal acceleration, then tilt ------------
    #    T from the horizontal pair; zeta = 4/(2*sqrt(6)) = 0.816 for ANY T.
    T  = min(max(2.0 * perr / max(0.05, verr), T_GAIN_MIN), T_GAIN_MAX)
    kv, kp = 4.0 / T, 6.0 / T ** 2
    a_h_vec = u_r[1:3] + kv * dve + kp * dp        # single self-consistent vector
    a_h_dem = float(np.linalg.norm(a_h_vec))

    # plan's own vertical thrust component drives the tilt denominator
    a_up_ref = max(1.0, abs(float(u_r[0])))
    tilt_dem = math.degrees(math.atan2(a_h_dem, a_up_ref))

    # ---- 4. THE tilt limits: one cap, one floor, one rate -------------------
    alt   = float(error[0]) + TARGET_ALT
    t_cap = min(max(alt / TILT_CAP_SLOPE, TILT_CAP_LO), TILT_CAP_HI)
    tilt  = min(tilt_dem, t_cap)

    #    terminal taper: shrink toward vertical, but never below the cap floor
    #    while horizontal speed remains (this is the 15(a) safety fix)
    if alt < TILT_TERM_ALT:
        taper = max(0.0, alt / TILT_TERM_ALT)
        tilt  = tilt * taper if np.linalg.norm(vel[1:3]) <= 1.0 else \
                max(tilt * taper, min(tilt, TILT_CAP_LO))

    #    rate limit: the physical "can start and can stop" bound. MUST KEEP.
    alpha  = math.radians(cap.alpha_max)                 # deg/s^2 -> rad/s^2
    slew   = math.degrees(math.radians(SLEW_LAG_DEG) *
                          math.sqrt(max(alpha, 1e-9) * SLEW_KP_REF))
    slew   = min(max(slew, TILT_CAP_LO), 14.0)           # safety clamp
    tilt   = st.tilt + min(max(tilt - st.tilt, -slew * dt), slew * dt)
    tilt   = min(max(tilt, 0.0), t_cap)                  # cap AFTER rate limit
    st.tilt = tilt

    # ---- 5. azimuth: from the SAME vector that set the magnitude -----------
    if a_h_dem > 1e-9:
        st.az = (a_h_vec / a_h_dem).copy()
    elif st.az is None:
        st.az = np.array([0.0, 0.0])                     # degenerate: fly vertical

    # ---- 6. THE vertical magnitude: a real vertical law, not a projection ---
    #    `a_ver_need` is the plan's own identity (verified: node 76 -> 25.51 vs 25.49).
    a_ver_need = G0 + (float(vel[0]) ** 2 - AVER_TGT_VZ ** 2) / \
                 (2.0 * max(0.5, float(error[0])))
    a_up = u_r[0] + K_VEL * (v_r[0] - vel[0]) + K_POS * (x_r[0] - error[0])
    a_up = min(a_up, a_ver_need)          # never brake harder than "just land"
    a_up = max(a_up, vertical_floor(cap, mass, u_i))     # never free-fall
    a_up = max(0.0, min(a_up, 0.99 * G0))                # hard: < hover

    # ---- 7. command: constructed ONCE, from (tilt, az, a_up) ---------------
    tr    = math.radians(tilt)
    a_cmd = np.array([a_up, a_up * math.tan(tr) * st.az[0],
                            a_up * math.tan(tr) * st.az[1]])
    mag   = float(np.linalg.norm(a_cmd))
    if mag > cap.a_cap:
        a_cmd *= cap.a_cap / mag                         # never exceed authority

    # ---- 8. attitude target: same (tilt, az), one construction -------------
    direction = np.array([math.cos(tr), math.sin(tr) * st.az[0],
                                         math.sin(tr) * st.az[1]])
    direction /= max(1e-9, float(np.linalg.norm(direction)))
    direction  = rate_limit_direction(direction, st, cap.alpha_max, dt)

    return a_cmd, direction, st


def vertical_floor(cap, mass, u_i):
    """The bound that stops free-fall. Collapsed from _gfold_floor_effective."""
    hover   = G0 * mass / max(1.0, cap.thrust_true)
    base    = min(0.16, THROTTLE_MARGIN * hover) * cap.a_cap
    plan_m  = float(np.linalg.norm(u_i))
    release = min(FLOOR_RELEASE_K * cap.a_cap, 1.02 * plan_m)
    return release if float(u_i[0]) - G0 < 0.0 else base
```

### 5.1 Behaviour that MUST be preserved (limits that prevent divergence)

These are not tunables; each one is the fix for a recorded crash. A rewrite that drops any
of them will diverge.

| # | Must keep | Current lines | Recorded failure it prevents |
|---|---|---|---|
| 1 | Thrust authority `available_thrust * t_scale / mass`, with online `t_scale` | 4002-4064, 3803-3865 | throttle floor above true hover -> rocket climbs to 409 m (4058-4062) |
| 2 | A **positive** lower bound on the vertical command | 3593, 4199-4219, 4322-4335 | 36/70 frames at zero throttle, free-fall 597->305 m (4155-4173) |
| 3 | Vertical command **strictly below** `g0` | 3533-3537, 4318 (`G0*0.99`) | net-up command -> never lands (359-372) |
| 4 | `a_up <= a_ver_need` (the plan's own identity) | 3574-3592 | the node-78 discretisation spike -> climb-away at 22 m (815-847) |
| 5 | Tilt **rate** limit derived from `alpha_max` | 3458-3485 | 169 deg/s reference vs 45 deg/s capability (3136-3140); binding in 34/123 frames |
| 6 | Tilt **cap** `clamp(alt/3, 5, 35)` | 3419-3422 | airframe past 90 deg -> 114 m/s lateral (481-485) |
| 7 | Terminal tilt floor while `vh > 1` | 3437-3456 | vh never killed, vertical-but-drifting touchdown (3428-3433) |
| 8 | Monotonic, `N-2`-capped plan index | 2751, 2798 | index rushing to `N-1` -> 5 s of "arrived" at 470 m (2759-2776) |
| 9 | Bandwidth cap: `kv <= 1.25` | 3296-3297 | deep instability PM = -102 deg (697-705) |
| 10 | `T >= T_GAIN_MIN` for the horizontal loop | 3289-3290 | PM < 0 (604-608) |
| 11 | Reference-direction **rate** limit | 3867-3956, 3692 | azimuth/direction rate 169 deg/s, asymmetric with tilt (3136-3140) |
| 12 | Azimuth hold when `|dp| < 1.5 m` | 3510, 3530-3537 | `atan2` of a near-zero vector |
| 13 | Correct target frame (`velocity=tgt_frame`) | 2369-2372 | whole-body 174.9 m/s phantom horizontal velocity -> infeasible plan, 817 m/s^2 commands (2340-2365) |
| 14 | Lead the reference by ~1.5 s | 2676-2699, 3182-3183 | 1.2-2.2 s airframe lag, never closed |
| 15 | Write actuators from the very first frame (`hold`) | 5521-5646 | 3.9 s uncontrolled free-fall after kOS handoff (5231-5250) |
| 16 | Game-pause / takeover guards | 5399-5415, 4991-5056 | false "kOS still owns throttle" abort while paused (5393-5397) |

### 5.2 What disappears entirely

- the old-law attitude path: `target_a_` (2858), `limit_tilt_dir` @3103, `tilt_cmd`
  derivation @3696-3697 — ~10 statements
- the tilt-first switch and its `try/except` fallback scaffolding (3166, 3663-3676)
- the `AVER_*` clamp (3563-3627) and its 8 constants — ~60 lines
- the `_tgo`/`_tgo_eff`/`_derate`/`_vhoriz`/`_tcorr_tgo`/`_tcorr_pos` block (3200-3239,
  3340-3343) — ~60 lines
- `_x_i_`/`v_i_`/`u_i_` forward sampling (2845, 2852)
- 122 `dbg_*` fields (~250 lines in `__init__`, 4794-4983 in `_dbg_cols`)
- 24 dead constants (Section 3.9)
- 22 always-empty/constant log columns
- `conic_clamp`'s horizontal branch (4336)

Estimated result: **~450-600 lines** of live control flow, down from 6076.

### 5.3 Recommended migration order (each independently verifiable)

1. Replace `av_up = |target_a|*cos(tilt)` (3563-3565) with a direct vertical law.
   Verify `av_up/av_plan -> 1` (currently p50 0.92, p90 1.72).
2. Delete the dead `target_a_` / `limit_tilt_dir`@3103 path and pin the tilt-first branch
   (removes the switch). Verify logs are byte-identical.
3. Delete the `_tgo` block. Verify logs are byte-identical.
4. Collapse the two tilt caps into one. **Requires a flight** — this is a behaviour change.
5. Strip diagnostics to a fixed minimal set.

Steps 1-3 are provably no-ops or localised; step 4 is the only one that must be flown.

---

## Appendix A — prior-report corrections

| Claim in prior report | Verified status |
|---|---|
| `ATTITUDE_ROOT_CAUSE_20261007.md` §3.1: "`_limit_ref_slew` is dead code, output overridden at 3616" | **OUTDATED.** True when written; step 18(a) moved the call to 3692. Now live (`ref_clip=1` in 21/123 frames). |
| Same report, "附带发现: `ref_clip` is sticky, 97.8%" | **FIXED** at 3894-3914 (`dbg_ref_clamped = 0.0` reset every frame). Measured 21/123 now, not 87/89. |
| Same report §3.2: "azimuth channel has no rate limit, 62.5% of frames exceed 14 deg/s" | **STILL TRUE** — `_limit_ref_slew` limits the 3-D direction, but there is no separate azimuth-rate term. Verified `tf_ahdir_cos` p50 = 0.976 but min = -1. |
| Same report §8: `check_py.ps1` FAILs with 10 pyright errors | Not re-run (out of scope); `check_cols_krpc.py` claims 149 columns — **confirmed 149**. |
| `NOTE_VS_CURRENT_AND_INTEGRATION_PLAN.md` §5: "`av_up/av_plan` p10=0.519 p50=0.920 p90=1.719 max=2.799" | **CONFIRMED** exactly on 212924 (p50 0.9202, p90 1.719, max 2.799). |
| Same §6: "`_amag` 删除（污染的来源）" | **CONFIRMED and quantified** — pre-clamp vertical PD is negative in 58.5% of frames, median -8.10. |
| Brief to this analysis: "`av_up == av_old` except in ~4% of frames" | **CORRECTION: 0.0%**, 0 differing frames in 391 across four flights. |
| Brief: "`a_cmd_h == a_cmd_mag*sin(tilt_cmd)` max deviation 0.003" | **CONFIRMED**: max 0.002555 (212924), 0.00293 (204325). |
| Brief: "`av_up == a_cmd_mag*cos(tilt_cmd)` max deviation 0.001" | **CONFIRMED**: max 0.001081 (212924), 0.000988 (204325). |
| `HANDOFF_20261007.md` §3 line map (2796/2998/3034/3078/3096/3195/3262/3462/3536/3585/3596/3624) | **OFFSET BY ~+60 LINES** — the file grew after that report. Correct anchors: 2857, 3059, 3095, 3166, 3184, 3238, 3289, 3480, 3563, 3593, 3612, 3651. |

## Appendix B — verification commands (read-only)

```
# invariant checks over any log
python - <<'PY'
import csv, math
rows=[r for r in csv.DictReader(l for l in open(
  r"log-B1040-9-gfold/gfold_log_20261007_212924.csv",
  encoding="utf-8",errors="replace") if not l.startswith("#"))]
F=lambda r,k:(lambda v: v if v==v else float("nan"))(float(r[k] or "nan"))
print("av_up!=av_old :", sum(1 for r in rows if abs(F(r,"av_up")-F(r,"av_old"))>1e-9))
print("a_cmd_up==av_up:", all(abs(F(r,"a_cmd_up")-F(r,"av_up"))<1e-9 for r in rows))
print("tilt_cmd==tf_cmd:", all(abs(F(r,"tilt_cmd")-F(r,"tf_cmd"))<1e-9 for r in rows))
print("av_need NaN   :", sum(1 for r in rows if F(r,"av_need")!=F(r,"av_need")), "/", len(rows))
PY
```
