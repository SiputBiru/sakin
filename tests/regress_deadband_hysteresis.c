/* SPDX-License-Identifier: 0BSD */
/* regress_deadband_hysteresis.c - implicit-state-machine regression test (F1)
 *
 * The gait engine has no explicit "mode" variable: the mode is derived every
 * tick from whether |vx|,|vy|,|omega| exceed the deadbands
 * (sakin__gait_update, sakin.h).  There is NO hysteresis on that transition,
 * and every non-moving tick calls sakin__settle(), which clears `primed`.
 *
 * This test drives the SAME mean velocity two ways:
 *   1. steadily just above the deadband  (6 mm/s)
 *   2. dithered around it every tick      (6, 4, 6, 4, ... mm/s)
 *
 * A state machine with a sticky/hysteretic moving latch would keep walking
 * through the 4 mm/s ticks, so the dithered cadence should stay close to the
 * steady one.  Without hysteresis the engine re-enters the prime state on
 * every other tick, so the cadence roughly halves and `primed` is cleared
 * hundreds of times.
 *
 * EXPECTED BEFORE A FIX: the cadence and primed-reset checks FAIL.
 * Build: cc -std=c99 -Wall -Wextra -Werror -pedantic regress_deadband_hysteresis.c -lm
 */
#include <stdio.h>
#include <math.h>

#define SAKIN_IMPLEMENTATION
#include "../sakin.h"

static int g_checks = 0, g_fails = 0;

#define CHECK(cond, msg)                                    \
  do {                                                      \
    ++g_checks;                                             \
    if (!(cond)) {                                          \
      ++g_fails;                                            \
      printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
    }                                                       \
  } while (0)

typedef struct {
  int step_advances;
  int primed_resets;
  sakin_real_t max_lift;
  sakin_real_t mean_offset_x;
} gait_stats_t;

/* Run 20 s @ 60 Hz. dither=0: vx = deadband+1. dither=1: 6,4,6,4,... */
static gait_stats_t run_case(int dither) {
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  gait_stats_t st;
  int i, l, prev_step, prev_primed;
  sakin_real_t sum_x = 0;

  sakin_init(&bot, NULL);
  sakin_gait_select(&bot, SAKIN_GAIT_TRIPOD);

  st.step_advances = 0;
  st.primed_resets = 0;
  st.max_lift = 0;
  prev_step = bot.step;
  prev_primed = bot.primed;

  for (i = 0; i < 1200; ++i) {
    sakin_real_t vx = dither
      ? ((i & 1) ? (bot.cfg.deadband_lin - 1.0f)
                 : (bot.cfg.deadband_lin + 1.0f))
      : (bot.cfg.deadband_lin + 1.0f);

    sakin_command(&bot, vx, 0.0f, 0.0f);
    sakin_update(&bot, 1.0f / 60.0f, a);

    for (l = 0; l < bot.cfg.leg_count; ++l)
      if (bot.offsets[l].z > st.max_lift) st.max_lift = bot.offsets[l].z;

    if (bot.step != prev_step) { ++st.step_advances; prev_step = bot.step; }
    if (prev_primed == 1 && bot.primed == 0) ++st.primed_resets;
    prev_primed = bot.primed;

    for (l = 0; l < bot.cfg.leg_count; ++l) sum_x += bot.offsets[l].x;
  }
  st.mean_offset_x = sum_x / (sakin_real_t)(1200 * bot.cfg.leg_count);
  return st;
}

int main(void) {
  gait_stats_t steady = run_case(0);
  gait_stats_t dith = run_case(1);

  printf("steady   : cadence=%d primed_resets=%d max_lift=%.2f meanX=%.3f\n",
         steady.step_advances, steady.primed_resets, steady.max_lift,
         steady.mean_offset_x);
  printf("dithered : cadence=%d primed_resets=%d max_lift=%.2f meanX=%.3f\n",
         dith.step_advances, dith.primed_resets, dith.max_lift,
         dith.mean_offset_x);

  /* Sanity: the steady above-deadband command must actually walk. */
  CHECK(steady.step_advances > 250,
        "steady above-deadband command advances steps at full rate");

  /* F1a: a dither with the SAME mean intent should not halve the cadence. */
  CHECK(dith.step_advances >= (steady.step_advances * 9) / 10,
        "dithered velocity keeps >=90% of steady gait cadence (needs hysteresis)");

  /* F1b: the moving latch should not be cleared on every off-tick. */
  CHECK(dith.primed_resets <= 10,
        "primed is not cleared repeatedly while the operator still commands motion");

  /* The defect is cadence/thrash, not a lift loss - keep this as a guard. */
  CHECK(dith.max_lift > steady.max_lift * 0.9f,
        "dithering does not collapse swing lift (documents current behaviour)");

  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
