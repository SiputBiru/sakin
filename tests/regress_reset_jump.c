/* SPDX-License-Identifier: 0BSD */
/* regress_reset_jump.c - F8: sakin_reset_pose must cancel an in-flight jump.
 *
 * reset_pose() resets offsets, endpoints, gait selection and the moving latch.
 * It used to leave jump_active/jump_phase untouched, so a reset issued during
 * a jump had the jump keep overriding the freshly reset gait.  This test
 * starts a jump, resets, and asserts the jump is gone and normal walking
 * resumes.
 *
 * EXPECTED BEFORE A FIX: the "cancels" checks FAIL (jump stays active).
 * Build: cc -std=c99 -Wall -Wextra -Werror -pedantic regress_reset_jump.c -lm
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

static int finite_angles(const sakin_leg_angles_t *a, int n) {
  int l;
  for (l = 0; l < n; ++l)
    if (!isfinite((double)a[l].coxa) || !isfinite((double)a[l].femur) ||
        !isfinite((double)a[l].tibia)) return 0;
  return 1;
}

int main(void) {
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  int i, ok = 1;

  sakin_init(&bot, NULL);
  sakin_jump_start(&bot);
  for (i = 0; i < 5; ++i) sakin_update(&bot, 1.0f / 60.0f, a);
  CHECK(sakin_jump_active(&bot), "jump is active before the reset");

  sakin_reset_pose(&bot);

  printf("after reset: active=%d phase=%d\n",
         sakin_jump_active(&bot), sakin_jump_phase(&bot));
  CHECK(!sakin_jump_active(&bot), "reset_pose cancels an in-flight jump");
  CHECK(sakin_jump_phase(&bot) == SAKIN_JUMP_NONE,
        "reset_pose clears the jump phase to NONE");

  /* Normal gait must resume from the reset stance. */
  sakin_command(&bot, 50.0f, 0.0f, 0.0f);
  for (i = 0; i < 60; ++i) {
    sakin_update(&bot, 1.0f / 60.0f, a);
    if (!finite_angles(a, bot.cfg.leg_count)) ok = 0;
  }
  CHECK(ok, "gait runs normally after a reset that cancelled the jump");

  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
