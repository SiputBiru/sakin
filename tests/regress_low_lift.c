/* SPDX-License-Identifier: 0BSD */
/* regress_low_lift.c - F3: low lift_height must still place feet ahead.
 *
 * The gait's "was this leg airborne last step?" test used to be the proxy
 * `next[leg].z > 1.0f`.  Any cfg.lift_height <= 1 mm silently disabled the
 * touchdown-ahead branch, so feet were only dragged back and never placed
 * ahead -- the robot could not stride.
 *
 * This test walks with lift_height = 0.5 mm and asserts that some foot offset
 * still goes POSITIVE in X (placed ahead at touchdown).  Before the fix the
 * maximum X offset stays around 0, so the first check fails.
 *
 * Build: cc -std=c99 -Wall -Wextra -Werror -pedantic regress_low_lift.c -lm
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
  sakin_real_t max_x; /* most-positive body-frame offset X = placed ahead */
  sakin_real_t max_z; /* highest swing apex actually reached */
  int step_advances;
} stats_t;

static stats_t walk(sakin_real_t lift) {
  sakin_config_t cfg;
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  stats_t st;
  int i, l, prev_step;

  sakin_config_default(&cfg);
  cfg.lift_height = lift;
  sakin_init(&bot, &cfg);
  sakin_gait_select(&bot, SAKIN_GAIT_TRIPOD);
  sakin_command(&bot, 100.0f, 0.0f, 0.0f);

  st.max_x = -1e9f;
  st.max_z = -1e9f;
  st.step_advances = 0;
  prev_step = bot.step;

  for (i = 0; i < 300; ++i) { /* 5 s @ 60 Hz */
    sakin_update(&bot, 1.0f / 60.0f, a);
    for (l = 0; l < bot.cfg.leg_count; ++l) {
      if (bot.offsets[l].x > st.max_x) st.max_x = bot.offsets[l].x;
      if (bot.offsets[l].z > st.max_z) st.max_z = bot.offsets[l].z;
    }
    if (bot.step != prev_step) { ++st.step_advances; prev_step = bot.step; }
  }
  return st;
}

int main(void) {
  stats_t lo = walk(0.5f);   /* the config that used to break */
  stats_t hi = walk(40.0f);  /* default */

  printf("lift=0.5 : max_x=%.2f max_z=%.2f steps=%d\n",
         (double)lo.max_x, (double)lo.max_z, lo.step_advances);
  printf("lift=40  : max_x=%.2f max_z=%.2f steps=%d\n",
         (double)hi.max_x, (double)hi.max_z, hi.step_advances);

  CHECK(lo.max_x > 3.0f,
        "low lift: feet are placed ahead at touchdown (explicit airborne state)");
  CHECK(lo.max_z >= 0.4f && lo.max_z <= 0.6f,
        "low lift: swing apex honours the configured lift_height");
  CHECK(hi.max_x > 3.0f,
        "default lift still places feet ahead (no regression)");
  CHECK(SAKIN__FABS(lo.max_x - hi.max_x) < 0.5f,
        "stride placement is independent of lift height");
  CHECK(lo.step_advances > 60 && hi.step_advances > 60,
        "gait still advances steps in both configurations");

  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
