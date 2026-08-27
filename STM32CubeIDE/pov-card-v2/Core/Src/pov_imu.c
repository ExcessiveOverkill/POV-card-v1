/*
 * pov_imu_update() — fixed-point POV badge position estimator
 * ============================================================
 * Call at 1 kHz.  ±8 G full-scale int16 input.
 * Output: velocity and position in Q20 fixed-point.
 *   1 m/s → out_vel_q20 ≈ 1,048,576  (= 1 << 20)
 *   1 m   → out_pos_q20 ≈ 1,048,576
 *
 * Integer arithmetic only.  No float, no division in hot path.
 *
 *
 * ── Why previous versions drifted during tilt ────────────────────────
 *
 * Any fixed-TC gravity LP filter creates a lag between the true gravity
 * projection and its estimate.  That lag is proportional to tilt rate:
 *
 *   lin_ax_error ≈ G × ω_tilt × TC
 *
 * At 1 rad/s tilt with TC = 512 ms:  error ≈ 4096 × 1 × 0.512 ≈ 2100 LSB
 *                                           = 2 × G_LSB of phantom accel
 *
 * That integrates directly into velocity.  "Massive drift" is correct.
 * Three-axis tracking and magnitude normalisation do not fix this; the
 * lag is a property of the TC, not the number of axes being tracked.
 *
 * The filter must be slow during shaking to stop shake energy corrupting
 * the gravity estimate.  But it must be fast during tilting to eliminate
 * the lag.  These look like conflicting requirements — until you notice
 * that physics already signals which regime you are in.
 *
 *
 * ── The fix: adaptive time constant ──────────────────────────────────
 *
 * When the badge is tilting (or stationary) with no linear translation,
 * the total accelerometer magnitude is exactly G regardless of orientation:
 *
 *   a_total = R × g_world   →   |a_total| = G   (no linear component)
 *
 * When the badge is shaking, linear acceleration adds in and |a_total| > G.
 * This gives a free, zero-cost signal to switch TC adaptively:
 *
 *   |a_total| ≈ G   →  pure orientation change → FAST TC (≈ 16 ms)
 *   |a_total| >> G  →  linear motion present   → SLOW TC (≈ 512 ms)
 *
 * Fast TC:  gravity estimate tracks orientation almost instantly.
 *           Lag drops from 2100 LSB to ~66 LSB at 1 rad/s.
 *           lin_ax ≈ 0 during tilt → no velocity drift.
 *
 * Slow TC:  gravity estimate is stable during shake.
 *           lin_ax correctly reflects the shake acceleration.
 *
 * The threshold is placed at ±~0.14G deviation from G, which corresponds
 * to about 0.28G of linear acceleration.  Any POV-useful shake exceeds
 * that threshold, so the two operating regimes cleanly match the two
 * physical scenarios.
 *
 * The adaptive TC also eliminates the need for the delta-based gravity snap
 * and magnitude normalisation from the previous version: the moment shaking
 * stops, |a_total| returns to G, fast TC engages, and the gravity estimate
 * converges to the new orientation within ~16 ms automatically.
 *
 *
 * ── ZUPT and tilt ────────────────────────────────────────────────────
 *
 * With fast TC active, lin_ax ≈ lag_error which at normal hand tilt rates
 * (< 3 rad/s) is below the ZUPT threshold.  ZUPT fires and prevents
 * residual lag from accumulating into velocity:
 *
 *   max tilt rate before ZUPT stops firing:
 *   ω_max = ZUPT_THRESH_LSB / (TC_fast_s × G_LSB)
 *          = 209 / (0.016 × 4096)  ≈  3.2 rad/s  (≈ 183 °/s)
 *
 * Ordinary tilting is well below this.  For simultaneous tilt + vigorous
 * shake, some transient error is unavoidable without a gyroscope; it clears
 * once shaking stops and ZUPT fires.
 *
 *
 * ── Column mapping example ───────────────────────────────────────────
 *
 *  #define HALF_STROKE_Q20  83886   // 0.08 m × 2^20; tune to your shake
 *  int col = (int)((pos + HALF_STROKE_Q20) * NUM_COLS
 *                  / (2 * HALF_STROKE_Q20));
 *  if (col < 0)         col = 0;
 *  if (col >= NUM_COLS) col = NUM_COLS - 1;
 *
 *
 * ── Scale derivation ─────────────────────────────────────────────────
 *
 *  1 LSB = 8 × 9.81 / 32768 = 0.002394 m/s²
 *  Velocity Q20 increment per sample per LSB:
 *      0.002394 × 0.001 s × 2^20 = 2.511
 *  Approximated as (x<<1)+(x>>1) = x×2.5    (0.44% error)
 *  Position increment: vel>>10 ≈ vel/1000    (2.34% error)
 *
 *
 * ── Overflow budget (int32, worst-case ±8 G) ─────────────────────────
 *
 *  grav_acc : ±32767 × 512             ≈ ±16.8 M  (int32 max ±2147 M)  ✓
 *  Fast update boost: error << 5       ≈ ±1.05 M, pushes grav_acc to
 *                                        ±17.8 M                        ✓
 *  vel      : ±32767 × 2.5 × 2048     ≈ ±167.8 M                       ✓
 *  pos      : ±167.8 M × 8192         ≈ ±1375 M                        ✓
 *  total_sq3: ±(32767>>3)² × 3        ≈ ±201 M                         ✓
 *
 *
 * ── Tuning guide ─────────────────────────────────────────────────────
 *
 *  GRAV_SHIFT_SLOW   Gravity TC during shake = 2^N ms.
 *                    9 → 512 ms.  Must be >> your shake period.
 *
 *  GRAV_SHIFT_FAST   Gravity TC during tilt/rest = 2^N ms.
 *                    4 → 16 ms.  Smaller = faster tilt tracking but more
 *                    susceptible to brief transients near the G boundary.
 *                    Maximum safe tilt rate before lag exceeds ZUPT threshold:
 *                    ω_max = ZUPT_THRESH_LSB / (2^N ms × G_LSB / 1000).
 *
 *  NEAR_G_BAND       Near-G detection band, in total_sq3 units.
 *                    Switch from slow→fast TC when the total squared
 *                    magnitude (scaled by >>3) is within this band of G_SQ3.
 *                    20000 ≈ ±14% of G → linear threshold ≈ 0.28G.
 *                    Raise to catch faster tilts; lower to protect against
 *                    gentle shaking bleeding into gravity.
 *
 *  VEL_SHIFT         Velocity leak TC = 2^N ms.  11 → 2048 ms.
 *
 *  POS_SHIFT         Position leak TC = 2^N ms.  13 → 8192 ms.
 *
 *  ZUPT_THRESH_LSB   lin_ax stillness threshold for velocity bleed.
 *                    209 ≈ 0.5 m/s² at ±8G 16-bit.
 *
 *  ZUPT_HOLD_MS      Debounce window [samples = ms].  Must exceed the
 *                    acceleration zero-crossing duration at your max shake
 *                    rate (~16 ms at 2 Hz).
 *
 *  ZUPT_BLEED_SHIFT  vel × (1 − 1/2^N) per sample while ZUPT is active.
 *                    3 → 12.5% per sample, < 1% residual in ~35 ms.
 */

#include <stdint.h>

/* ── Gravity filter ──────────────────────────────────── */
#define GRAV_SHIFT_SLOW   9     /* TC ≈  512 ms  — shake protection      */
#define GRAV_SHIFT_FAST   4     /* TC ≈   16 ms  — tilt tracking         */

/* 1G in raw LSB at ±8G 16-bit: 32768/8 = 4096                           */
#define G_LSB             4096

/* G squared in the >>3-scaled magnitude domain: (G_LSB>>3)^2 = 512^2    */
#define G_SQ3             ((int32_t)((G_LSB>>3)*(G_LSB>>3)))   /* 262144 */

/* Band around G_SQ3 for near-G detection (see tuning guide above)        */
#define NEAR_G_BAND       20000

/* ── Velocity integrator (Q20: 1 m/s = 1<<20) ─────────── */
#define VEL_SHIFT         11    /* leaky decay, TC ≈ 2048 ms              */
#define VEL_DT_SHIFT      10    /* vel→pos: ÷1024 ≈ ÷1000                */

/* ── Position integrator (Q20: 1 m = 1<<20) ───────────── */
#define POS_SHIFT         13    /* leaky decay, TC ≈ 8192 ms              */

/* ── ZUPT ──────────────────────────────────────────────── */
#define ZUPT_THRESH_LSB  209    /* ≈ 0.5 m/s² at ±8G 16-bit              */
#define ZUPT_HOLD_MS      40    /* debounce window [samples = ms]         */
#define ZUPT_BLEED_SHIFT   3    /* vel × 7/8 per sample when active       */


void pov_imu_update(int16_t ax, int16_t ay, int16_t az,
                    int32_t *out_vel_q20, int32_t *out_pos_q20)
{
    static int32_t  grav_acc_x = 0;   /* X gravity accumulator [LSB×512] */
    static int32_t  grav_acc_y = 0;   /* Y gravity accumulator           */
    static int32_t  grav_acc_z = 0;   /* Z gravity accumulator           */
    static int32_t  vel        = 0;   /* badge-X velocity     [Q20 m/s]  */
    static int32_t  pos        = 0;   /* badge-X position     [Q20 m]    */
    static uint16_t still      = 0;   /* ZUPT debounce counter           */
    static uint8_t  init       = 0;

    /* Seed gravity accumulators from the first real reading so they
       don't need ~3 × GRAV_SHIFT_SLOW ≈ 1.5 s to ramp up from zero.  */
    if (!init) {
        grav_acc_x = (int32_t)ax << GRAV_SHIFT_SLOW;
        grav_acc_y = (int32_t)ay << GRAV_SHIFT_SLOW;
        grav_acc_z = (int32_t)az << GRAV_SHIFT_SLOW;
        init = 1;
    }

    /* ── Step 1: Near-G detection ────────────────────────────────────────
     *
     * Scale all axes down by >>3 before squaring so the sum of squares
     * stays within int32 (max single-axis: 4096>>3 = 512, sq = 262144;
     * worst-case sum of three at 8G full-scale: 4095² × 3 ≈ 50 M << 2.1B).
     *
     * total_sq3 ≈ G_SQ3 means the measured vector is almost entirely
     * gravity: the badge is tilting or stationary, not shaking.
     * ──────────────────────────────────────────────────────────────────── */
    {
        int32_t ax3 = (int32_t)ax >> 3;
        int32_t ay3 = (int32_t)ay >> 3;
        int32_t az3 = (int32_t)az >> 3;
        int32_t total_sq3 = ax3*ax3 + ay3*ay3 + az3*az3;

        /* ── Step 2: Adaptive gravity LP update ──────────────────────────
         *
         * Both modes use the same accumulator (storing grav_est × 2^SLOW).
         * In fast mode the error term is amplified by 2^(SLOW−FAST) = 32,
         * boosting the effective alpha from 1/512 to 1/32:
         *
         *   normal:  grav_acc += error        →  Δgrav_est = error/512
         *   fast:    grav_acc += error << 5   →  Δgrav_est = error/16
         *
         * Fast TC ≈ 16 ms means a 90° reorientation (e.g. flat→upright)
         * settles in about 3 × 16 ms ≈ 50 ms rather than ~1.5 s.
         * ──────────────────────────────────────────────────────────────── */
        int32_t ex = (int32_t)ax - (grav_acc_x >> GRAV_SHIFT_SLOW);
        int32_t ey = (int32_t)ay - (grav_acc_y >> GRAV_SHIFT_SLOW);
        int32_t ez = (int32_t)az - (grav_acc_z >> GRAV_SHIFT_SLOW);

        int32_t g_err = total_sq3 - G_SQ3;
        if (g_err > -(int32_t)NEAR_G_BAND && g_err < (int32_t)NEAR_G_BAND) {
            /* Near-G: tilt/rest mode — fast gravity tracking */
            grav_acc_x += ex << (GRAV_SHIFT_SLOW - GRAV_SHIFT_FAST);
            grav_acc_y += ey << (GRAV_SHIFT_SLOW - GRAV_SHIFT_FAST);
            grav_acc_z += ez << (GRAV_SHIFT_SLOW - GRAV_SHIFT_FAST);
        } else {
            /* Far-from-G: shake mode — slow gravity tracking */
            grav_acc_x += ex;
            grav_acc_y += ey;
            grav_acc_z += ez;
        }
    }

    /* ── Step 3: Linear acceleration on badge X axis ─────────────────── */
    int32_t lin_ax = (int32_t)ax - (grav_acc_x >> GRAV_SHIFT_SLOW);

    /* ── Step 4: Leaky velocity integration ─────────────────────────────
     *  Increment ≈ lin_ax × 2.511 Q20  →  approximated as lin_ax × 2.5
     *  via (lin_ax<<1)+(lin_ax>>1).  Error: 0.44%.
     * ──────────────────────────────────────────────────────────────────── */
    vel += (lin_ax << 1) + (lin_ax >> 1);
    vel -= vel >> VEL_SHIFT;

    /* ── Step 5: Debounced ZUPT — velocity bleed ─────────────────────────
     *
     * In tilt/rest mode (step 2), gravity tracks quickly so lin_ax ≈ 0
     * and ZUPT fires within ZUPT_HOLD_MS.  This means the same ZUPT that
     * handles "badge stopped after shaking" also handles "badge being
     * tilted" — both produce near-zero lin_ax once gravity is up to date.
     *
     * The debounce prevents triggering at the natural acceleration
     * zero-crossing at mid-swing during a shake.
     * ──────────────────────────────────────────────────────────────────── */
    if (lin_ax > -ZUPT_THRESH_LSB && lin_ax < ZUPT_THRESH_LSB) {
        if (still < (uint16_t)ZUPT_HOLD_MS + 1u) still++;
        if (still > ZUPT_HOLD_MS) {
            vel -= vel >> ZUPT_BLEED_SHIFT;
        }
    } else {
        still = 0;
    }

    /* ── Step 6: Leaky position integration ──────────────────────────── */
    pos += vel >> VEL_DT_SHIFT;
    pos -= pos >> POS_SHIFT;

    if (out_vel_q20) *out_vel_q20 = vel;
    if (out_pos_q20) *out_pos_q20 = pos;
}
