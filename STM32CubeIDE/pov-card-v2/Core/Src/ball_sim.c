/*
 * ball_sim_tick() — 1-D bouncing ball simulator, 32 LEDs, 1 kHz
 * ==============================================================
 * Call exactly once per millisecond (1 kHz).
 *
 *  accel_in : int16_t axis acceleration.
 *             Positive  → ball accelerates toward LED 31.
 *             Negative  → ball accelerates toward LED  0.
 *
 *  Returns  : uint8_t LED index to illuminate (0 – 31).
 *
 * ── Why Q16 instead of Q10 ──────────────────────────────────────
 * At 1 kHz with Q10, 1 unit of constant friction = ~1 LED/second —
 * large enough to stall the ball against any small acceleration and
 * cause the jittery snapping you observed.  Moving to Q16 (1 unit =
 * 1/65536 of a LED) makes the same constant term 64× weaker, which
 * is invisible.  Better still: the bounce math already converges to
 * zero on its own (1 × 3 >> 2 = 0), so no constant friction term is
 * needed at all, and the dead-zone stiction disappears completely.
 *
 * Physics are Q16 fixed-point throughout (65 536 units = 1 LED).
 * No floating-point variables or arithmetic are used.
 * Relies on arithmetic (sign-extending) right-shift — standard on
 * ARM/GCC (STM32 etc.) and all common 32-bit toolchains.
 *
 * ── Quick-tune constants ─────────────────────────────────────────
 *  BP_VEL_MAX    Top speed, Q16 units/tick.
 *                8192  = 0.125 LED/ms → ~250 ms to cross the strip.
 *                16384 = 0.250 LED/ms → ~125 ms (snappier).
 *                4096  = 0.063 LED/ms → ~500 ms (very heavy).
 *
 *  BP_ACCEL_SHR  Input scale-down.  8 → full ±32 768 gives ±128
 *                vel/tick, reaching VEL_MAX in ~64 ticks at 8192.
 *                Decrease (e.g. 6) for a more reactive ball.
 *
 *  BP_DAMP_SHR   Proportional drag: vel -= vel >> this, each tick.
 *                12 → ~78 % speed retained after 1 s (low friction).
 *                11 → ~61 %  (moderate).
 *                13 → ~89 %  (near-frictionless / icy).
 *
 *  BP_BOUNCE     Wall restitution numerator (denominator = 4).
 *                3 → 75 % retained, ball bounces ~5 times from max.
 *                2 → 50 %  (dead thud, 3 bounces).
 *                4 would be perfectly elastic — avoid, ball won't stop.
 * ─────────────────────────────────────────────────────────────────
 */

#include <stdint.h>

#define BP_FRAC       16
#define BP_ONE        (1 << BP_FRAC)            /* 65536 = 1 LED        */
#define BP_POS_MAX    ((int32_t)31 * BP_ONE)    /* 2031616 ≡ LED 31     */
#define BP_VEL_MAX    ((int32_t)8192*32)           /* 0.125 LED/ms         */
#define BP_ACCEL_SHR  5                         /* input scale-down     */
#define BP_DAMP_SHR   14                        /* proportional drag    */
#define BP_BOUNCE     3                         /* restitution × 3/4    */

uint16_t ball_sim_tick(int16_t accel_in)
{
    static int32_t pos = (int32_t)16 * BP_ONE;  /* start at LED 16      */
    static int32_t vel = 0;

    /* ── 1. Acceleration ──────────────────────────────────────────────
     * Cast before shift so the full int16 sign range is preserved.    */
    vel += (int32_t)accel_in >> BP_ACCEL_SHR;

    /* ── 2. Speed limit ───────────────────────────────────────────────*/
    if      (vel >  BP_VEL_MAX) vel =  BP_VEL_MAX;
    else if (vel < -BP_VEL_MAX) vel = -BP_VEL_MAX;

    /* ── 3. Proportional damping ──────────────────────────────────────
     * vel -= vel >> DAMP_SHR each tick.  Arithmetic right-shift makes
     * this symmetric for both positive and negative velocity.
     *
     * No constant friction term: at Q16, residual velocities below the
     * proportional threshold are sub-pixel (< 4096/65536 ≈ 0.06 LED/ms)
     * and the bounce attenuator naturally converges vel to zero on its
     * own (1 × 3 >> 2 = 0), so a friction floor is unnecessary.       */
    vel -= vel >> BP_DAMP_SHR;

    /* ── 4. Integrate position ────────────────────────────────────────*/
    pos += vel;

    /* ── 5. Bouncy walls ──────────────────────────────────────────────
     * Direction guard prevents double-bounce when held against a wall.
     * Left  wall: reflect then attenuate  → result is always positive.
     * Right wall: attenuate then reflect  → result is always negative. */
    if (pos <= 0) {
        pos = 0;
        if (vel < 0) {
            vel = -vel;
            vel = (vel * BP_BOUNCE) >> 2;
        }
    } else if (pos >= BP_POS_MAX) {
        pos = BP_POS_MAX;
        if (vel > 0) {
            vel = (vel * BP_BOUNCE) >> 2;
            vel = -vel;
        }
    }


    return (uint16_t)(pos >> (BP_FRAC - 4));
}

void ball_to_leds(uint16_t led_pos, uint8_t brightness[32])
{
    uint8_t idx  = (uint8_t)(led_pos >> 4);    /* LED index,    0 – 31 */
    uint8_t frac = (uint8_t)(led_pos & 0xF);   /* sub-LED frac, 0 – 15 */

    brightness[idx] = 15 - frac;
    if (frac && idx < 31)
        brightness[idx + 1] = frac;
}
