/* Core tests: (1) the PGR law takes a unicycle with the wheelchair footprint through a 0.80 m door from oblique
 * starts without the footprint touching the jambs; (2) door detection on a synthetic noisy scan. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pgr_door.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static double gauss(unsigned *s) {
    double u = 0;
    for (int i = 0; i < 12; i++) { *s = *s * 1664525u + 1013904223u; u += (*s >> 8) / 16777216.0; }
    return u - 6.0;
}

/* footprint corner inside a jamb box? */
static int touches(const pgrd_params *p, pgrd_pose q, double W) {
    double xs[4] = {p->xmin, p->xmax, p->xmax, p->xmin}, ys[4] = {-p->hw, -p->hw, p->hw, p->hw};
    for (int i = 0; i < 4; i++) {
        double x = q.x + cos(q.th) * xs[i] - sin(q.th) * ys[i], y = q.y + sin(q.th) * xs[i] + cos(q.th) * ys[i];
        if (fabs(x) <= p->wall_t / 2 && fabs(y) >= W / 2) return 1;
    }
    /* jamb corners inside the footprint */
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2) {
            double dx = sx * p->wall_t / 2 - q.x, dy = sy * W / 2 - q.y;
            double u = cos(q.th) * dx + sin(q.th) * dy, v = -sin(q.th) * dx + cos(q.th) * dy;
            if (u > p->xmin && u < p->xmax && fabs(v) < p->hw) return 1;
        }
    return 0;
}

static void test_law(void) {
    pgrd_params p;
    pgrd_default_params(&p);
    const double W = 0.80, dt = 0.02;
    const double starts[][3] = {{-3.5, 0.0, 0.0}, {-3.2, 1.2, -0.6}, {-3.6, -1.4, 0.8}, {-3.0, 1.0, 0.0}, {-4.0, -0.6, -0.5}};
    for (int s = 0; s < 5; s++) {
        pgrd_ctrl c;
        pgrd_ctrl_init(&c, &p);
        pgrd_pose q = {starts[s][0], starts[s][1], starts[s][2]};
        double v = 0, w = 0, vc, wc;
        int st = 0, hit = 0, k = 0;
        for (; k < 3000 && st != PGRD_DONE; k++) {
            if (k % 5 == 0) st = pgrd_ctrl_step(&c, q, v, &vc, &wc);     /* 10 Hz control, 50 Hz integration */
            v += fmax(-0.8 * dt, fmin(0.8 * dt, vc - v));                /* acceleration limits */
            w += fmax(-1.5 * dt, fmin(1.5 * dt, wc - w));
            q.x += v * cos(q.th) * dt; q.y += v * sin(q.th) * dt; q.th = pgrd_wrap(q.th + w * dt);
            hit |= touches(&p, q, W);
        }
        CHECK(st == PGRD_DONE, "start %d did not finish", s);
        CHECK(!hit, "start %d touched a jamb", s);
        printf("law: start %d -> %s in %.1f s%s\n", s, st == PGRD_DONE ? "through" : "NOT through", k * dt, hit ? " (HIT)" : "");
    }
}

/* kinematic closed loop for one chassis; counts yaw-rate reversals before the door ("pose re-adjustments") */
static void run_chassis(int chassis, const char *name, double W, const double (*starts)[3], int ns, int expect_all) {
    pgrd_params p;
    pgrd_default_params_for(&p, chassis);
    const double dt = 0.02, L = 0.5, dmax = 0.6109;
    int ok_all = 1;
    for (int s = 0; s < ns; s++) {
        pgrd_ctrl c;
        pgrd_ctrl_init(&c, &p);
        pgrd_pose q = {starts[s][0], starts[s][1], starts[s][2]};
        double vx = 0, vy = 0, w = 0, cx = 0, cy = 0, cw = 0, delta = 0, last_sign = 0;
        int st = 0, hit = 0, k = 0, reversals = 0, stops = 0, back = 0;
        for (; k < 4000 && st != PGRD_DONE; k++) {
            if (k % 5 == 0) st = pgrd_ctrl_step_body(&c, q, hypot(vx, vy), &cx, &cy, &cw);
            vx += fmax(-0.8 * dt, fmin(0.8 * dt, cx - vx));
            vy += fmax(-0.8 * dt, fmin(0.8 * dt, cy - vy));
            if (chassis == PGRD_ACKERMANN) {              /* steering angle with a rate limit, yaw from the bicycle model */
                double dt_goal = fabs(vx) > 0.02 ? atan(L * cw / vx) : delta;
                dt_goal = fmax(-dmax, fmin(dmax, dt_goal));
                delta += fmax(-1.5 * dt, fmin(1.5 * dt, dt_goal - delta));
                w = vx * tan(delta) / L;
            } else {
                w += fmax(-1.5 * dt, fmin(1.5 * dt, cw - w));
            }
            q.x += (vx * cos(q.th) - vy * sin(q.th)) * dt;
            q.y += (vx * sin(q.th) + vy * cos(q.th)) * dt;
            q.th = pgrd_wrap(q.th + w * dt);
            hit |= touches(&p, q, W);
            if (q.x + p.xmax < 0.0) {                     /* before the front reaches the wall */
                double sg = fabs(w) > 0.05 ? (w > 0 ? 1 : -1) : 0;
                if (sg != 0 && last_sign != 0 && sg != last_sign) reversals++;
                if (sg != 0) last_sign = sg;
                if (k > 100 && hypot(vx, vy) < 0.02) stops++;
                if (vx < -0.02) back++;
            }
        }
        int one_go = st == PGRD_DONE && !hit && stops == 0 && back == 0;
        ok_all &= one_go;
        printf("%-10s door %.2f start (%5.2f,%5.2f,%5.2f): %s in %4.1f s, yaw-rate reversals before the door: %d%s\n", name, W,
               starts[s][0], starts[s][1], starts[s][2], one_go ? "one go" : "FAILED", k * dt, reversals, hit ? " (HIT)" : "");
    }
    if (expect_all) CHECK(ok_all, "%s: not all starts passed in one go", name);
}

static void test_chassis(void) {
    const double st_diff[][3] = {{-3.5, 0.0, 0.0}, {-3.2, 1.2, -0.6}, {-3.6, -1.4, 0.8}, {-3.0, 1.0, 0.0}};
    const double st_ack[][3] = {{-4.5, 0.0, 0.0}, {-4.5, 1.0, -0.3}, {-4.8, -1.2, 0.4}, {-4.2, 0.6, 0.0}};
    const double st_4ws[][3] = {{-3.5, 0.0, 0.0}, {-3.2, 1.2, -0.6}, {-3.6, -1.4, 0.8}, {-3.0, 1.0, 0.0}};
    run_chassis(PGRD_DIFF, "diff", 0.80, st_diff, 4, 1);
    run_chassis(PGRD_ACKERMANN, "ackermann", 0.80, st_ack, 4, 1);
    run_chassis(PGRD_4WS, "4ws", 0.80, st_4ws, 4, 1);
}

static void test_detect(void) {
    pgrd_detect_params dp;
    pgrd_detect_default(&dp);
    unsigned seed = 7u, ns = 99u;
    const double W = 0.85;
    pgrd_pose robot = {-2.0, 0.3, 0.2}, lidar = pgrd_compose(robot, (pgrd_pose){0.70, 0.0, 0.0});
    static double pts[2 * 2000];
    int n = 0;
    for (double y = -3.0; y < 3.0; y += 0.01) {                       /* near wall face at x = -0.06 */
        if (fabs(y) < W / 2) continue;
        pgrd_pose r = pgrd_compose(pgrd_inverse(robot), (pgrd_pose){-0.06, y, 0.0});
        pgrd_pose rl = pgrd_compose(pgrd_inverse(lidar), (pgrd_pose){-0.06, y, 0.0});
        double rr = hypot(rl.x, rl.y), a = atan2(rl.y, rl.x);
        if (fabs(a) > 2.356 || rr > 8.0) continue;
        double scale = (rr + 0.02 * gauss(&ns)) / rr;                 /* 2 cm range noise along the beam */
        pts[2 * n] = 0.70 + rl.x * scale; pts[2 * n + 1] = rl.y * scale;
        (void)r;
        n++;
    }
    pgrd_pose truth = pgrd_inverse(robot), prior = truth, door;
    prior.x += 0.1; prior.y -= 0.1; prior.th += 0.1;
    double width;
    int ok = pgrd_detect(&dp, pts, n, prior, NULL, NULL, 0, 0.7, 0.0, &seed, &door, &width);
    CHECK(ok, "door not detected");
    if (ok) {
        double e = hypot(door.x - truth.x, door.y - truth.y);
        printf("detect: centre error %.2f cm, heading error %.2f deg, width %.3f m (true %.2f)\n", 100 * e,
               fabs(pgrd_wrap(door.th - truth.th)) * 180 / M_PI, width, W);
        CHECK(e < 0.03, "centre error %.3f", e);
        CHECK(fabs(width - W) < 0.05, "width %.3f", width);
    }
}

int main(void) {
    test_law();
    test_chassis();
    test_detect();
    printf(fails ? "%d FAILURE(S)\n" : "all tests passed\n", fails);
    return fails ? 1 : 0;
}
