/* SPDX-License-Identifier: 0BSD */
/* regress_large_dt.c - implicit-state-machine regression test (F2)
 *
 * sakin_update() clamps NEGATIVE dt but not non-finite or huge dt
 * (sakin.h: "if (dt < 0) dt = 0;").  Inside sakin__gait_update the phase clock
 * advances with
 *
 *     s->t_in_step += dt * 1000.0f;
 *     while (s->t_in_step >= s->tran_ms) { ... }
 *
 * Consequences exercised here:
 *   A. large finite dt is handled by internal sub-stepping (this PASSES and is
 *      kept as a regression guard - one 100 ms tick == six 16.67 ms ticks);
 *   B. a huge finite dt costs O(dt / tran_ms) loop iterations (reported);
 *   C. NaN dt is NOT rejected and poisons offsets + output angles (FAILS);
 *   D. +inf dt would make the while loop run forever (NOT executed here - it
 *      would hang; documented so a fix is expected to clamp/cap it).
 *
 * EXPECTED BEFORE A FIX: the non-finite-dt check FAILS.
 * Build: cc -std=c99 -Wall -Wextra -Werror -pedantic regress_large_dt.c -lm
 */
#include <stdio.h>
#include <math.h>
#include <time.h>

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

static sakin_t warm_bot(void) {
  sakin_t b;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  int i;
  sakin_init(&b, NULL);
  sakin_command(&b, 100.0f, 0.0f, 0.5f);
  for (i = 0; i < 90; ++i) sakin_update(&b, 1.0f / 60.0f, a);
  return b;
}

static sakin_real_t avg_offset_diff(const sakin_t *x, const sakin_t *y) {
  sakin_real_t d = 0;
  int l;
  for (l = 0; l < x->cfg.leg_count; ++l) {
    d += SAKIN__FABS(x->offsets[l].x - y->offsets[l].x);
    d += SAKIN__FABS(x->offsets[l].y - y->offsets[l].y);
    d += SAKIN__FABS(x->offsets[l].z - y->offsets[l].z);
  }
  return d / (sakin_real_t)x->cfg.leg_count;
}

static int state_is_finite(const sakin_t *b, const sakin_leg_angles_t *a) {
  int l;
  if (!isfinite((double)b->t_in_step)) return 0;
  for (l = 0; l < b->cfg.leg_count; ++l) {
    if (!isfinite((double)b->offsets[l].x) ||
        !isfinite((double)b->offsets[l].y) ||
        !isfinite((double)b->offsets[l].z)) return 0;
    if (!isfinite((double)a[l].coxa) ||
        !isfinite((double)a[l].femur) ||
        !isfinite((double)a[l].tibia)) return 0;
  }
  return 1;
}

int main(void) {
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  sakin_t A, B;

  /* A. large finite dt must equal the equivalent sequence of small ticks. */
  A = warm_bot();
  B = warm_bot();
  sakin_update(&A, 0.100f, a);
  {
    int i;
    for (i = 0; i < 6; ++i) sakin_update(&B, 0.100f / 6.0f, a);
  }
  printf("large finite dt equivalence: avgOffsetDiff=%.3e (step A=%d B=%d)\n",
         (double)avg_offset_diff(&A, &B), A.step, B.step);
  CHECK(avg_offset_diff(&A, &B) < 1e-3f,
        "one large dt tick matches equivalent small ticks (sub-stepping correct)");

  /* B. huge finite dt executes O(dt / tran_ms) work; report, stay finite. */
  A = warm_bot();
  {
    clock_t t0 = clock();
    sakin_update(&A, 600.0f, a); /* 600 s in one tick */
    printf("huge finite dt (600 s): %.1f ms of work, t_in_step=%.2f, finite=%d\n",
           (double)(clock() - t0) * 1000.0 / (double)CLOCKS_PER_SEC,
           (double)A.t_in_step, state_is_finite(&A, a));
  }
  /* correctness only; the cost is printed above as the demonstration */
  CHECK(state_is_finite(&A, a), "huge finite dt leaves a finite state");

  /* C. NaN dt is not rejected -> poisons offsets and output angles. */
  A = warm_bot();
  sakin_update(&A, (sakin_real_t)NAN, a);
  printf("NaN dt: offsets[0]=(%.2f %.2f %.2f) t_in_step=%.2f finite=%d\n",
         (double)A.offsets[0].x, (double)A.offsets[0].y, (double)A.offsets[0].z,
         (double)A.t_in_step, state_is_finite(&A, a));
  CHECK(state_is_finite(&A, a),
        "NaN dt must be rejected (offsets/angles stay finite)");

  /* D. +inf dt is NOT executed: t_in_step += inf never decreases under the
   *    while loop's -= tran_ms, so sakin_update() would spin forever.  A fix
   *    should clamp dt (e.g. dt = min(dt, 0.1f) and isfinite(dt)). */

  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
