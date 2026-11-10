/* pgr_door - see pgr_door.h.  License: Apache-2.0 */
#include "pgr_door.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PGRD_MAXPTS 4096

/* ------------------------------------------------------------------ SE(2) */
double pgrd_wrap(double a) { return atan2(sin(a), cos(a)); }

pgrd_pose pgrd_compose(pgrd_pose a, pgrd_pose b) {
    pgrd_pose r;
    double c = cos(a.th), s = sin(a.th);
    r.x = a.x + c * b.x - s * b.y;
    r.y = a.y + s * b.x + c * b.y;
    r.th = pgrd_wrap(a.th + b.th);
    return r;
}

pgrd_pose pgrd_inverse(pgrd_pose a) {
    pgrd_pose r;
    double c = cos(a.th), s = sin(a.th);
    r.x = -c * a.x - s * a.y;
    r.y = s * a.x - c * a.y;
    r.th = -a.th;
    return r;
}

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------ controller */
void pgrd_default_params(pgrd_params *p) {
    p->xmin = -0.25; p->xmax = 0.80; p->hw = 0.33;
    p->wall_t = 0.12;
    p->n = 3.0; p->k = 2.0;
    p->v_far = 0.6; p->v_near = 0.3; p->w_max = 1.0;
    p->pre = 0.0;           /* derived: 0.80 + 0.06 + 0.75 = 1.61 m for the wheelchair */
    p->exit_dist = 2.0;
    p->goal_tol = 0.4;
    p->dt = 0.1;
}

double pgrd_derived_pre(const pgrd_params *p) { return p->xmax + p->wall_t / 2.0 + 5.0 * p->v_near / p->k; }

void pgrd_ctrl_init(pgrd_ctrl *c, const pgrd_params *p) {
    c->p = *p;
    c->pre = p->pre > 0.0 ? p->pre : pgrd_derived_pre(p);
    c->stage = PGRD_APPROACH;
}

double pgrd_flow_heading(double x, double y, pgrd_pose well, double n) {
    double c = cos(well.th), s = sin(well.th);
    double dx = x - well.x, dy = y - well.y;
    double X = c * dx + s * dy, Y = -s * dx + c * dy;
    double t = X < 0.0 ? atan2(-n * Y, -X) : atan2(-Y, -X);
    return t + well.th;
}

int pgrd_ctrl_step(pgrd_ctrl *c, pgrd_pose pose, double v_meas, double *v_cmd, double *w_cmd) {
    const pgrd_params *p = &c->p;
    pgrd_pose well;
    if (c->stage == PGRD_APPROACH && pose.x > -c->pre - 0.05) c->stage = PGRD_EXIT;
    if (c->stage == PGRD_APPROACH) { well.x = -c->pre; well.y = 0.0; well.th = 0.0; }
    else { well.x = p->exit_dist; well.y = 0.0; well.th = 0.0; }
    if (c->stage == PGRD_EXIT && hypot(pose.x - well.x, pose.y - well.y) < p->goal_tol) c->stage = PGRD_DONE;
    if (c->stage == PGRD_DONE) { *v_cmd = 0.0; *w_cmd = 0.0; return PGRD_DONE; }

    double th = pgrd_flow_heading(pose.x, pose.y, well, p->n);
    double v = v_meas > 0.1 ? v_meas : 0.1;
    double xn = pose.x + v * cos(pose.th) * p->dt, yn = pose.y + v * sin(pose.th) * p->dt;
    double thn = pgrd_flow_heading(xn, yn, well, p->n);
    double ff = clampd(pgrd_wrap(thn - th) / p->dt, -p->w_max, p->w_max);        /* flow feed-forward */
    *w_cmd = clampd(ff + p->k * pgrd_wrap(th - pose.th), -p->w_max, p->w_max);
    int near_door = pose.x > -c->pre - 0.5 && pose.x < 1.0;
    *v_cmd = near_door ? p->v_near : p->v_far;
    return c->stage;
}

/* ------------------------------------------------------------------ door detection */
void pgrd_detect_default(pgrd_detect_params *p) {
    p->win = 1.3; p->thr = 0.025; p->iters = 60; p->min_inliers = 10;
    p->wmin = 0.55; p->wmax = 1.3; p->wall_t = 0.12;
    p->max_angle = 0.6; p->free_margin = 0.15;
}

static unsigned lcg(unsigned *s) { *s = *s * 1664525u + 1013904223u; return *s >> 8; }

static int cmp_d(const void *a, const void *b) {
    double d = *(const double *)a - *(const double *)b;
    return d < 0 ? -1 : (d > 0 ? 1 : 0);
}

/* nearest beam to direction a; returns its range */
static double beam_range(const double *ang, const double *rng, int nb, double a) {
    int best = 0;
    double bd = 1e9;
    for (int i = 0; i < nb; i++) {
        double d = fabs(pgrd_wrap(ang[i] - a));
        if (d < bd) { bd = d; best = i; }
    }
    return rng[best];
}

int pgrd_detect(const pgrd_detect_params *p, const double *pts, int npts, pgrd_pose prior,
                const double *beam_ang, const double *beam_rng, int nbeams, double ox, double oy,
                unsigned *rng_state, pgrd_pose *door, double *width) {
    static double px[PGRD_MAXPTS], py[PGRD_MAXPTS], s[PGRD_MAXPTS];
    static unsigned char inl[PGRD_MAXPTS], best_inl[PGRD_MAXPTS];
    int n = 0;
    for (int i = 0; i < npts && n < PGRD_MAXPTS; i++) {
        double x = pts[2 * i], y = pts[2 * i + 1];
        if (!isfinite(x) || !isfinite(y)) continue;
        if (hypot(x - prior.x, y - prior.y) < p->win) { px[n] = x; py[n] = y; n++; }
    }
    if (n < 12) return 0;

    /* RANSAC wall line */
    int best_cnt = -1;
    for (int it = 0; it < p->iters; it++) {
        int i = (int)(lcg(rng_state) % (unsigned)n), j = (int)(lcg(rng_state) % (unsigned)n);
        if (i == j) continue;
        double tx = px[j] - px[i], ty = py[j] - py[i], L = hypot(tx, ty);
        if (L < 0.05) continue;
        double nx = -ty / L, ny = tx / L;
        int cnt = 0;
        for (int q = 0; q < n; q++) {
            inl[q] = fabs((px[q] - px[i]) * nx + (py[q] - py[i]) * ny) < p->thr;
            cnt += inl[q];
        }
        if (cnt > best_cnt) { best_cnt = cnt; memcpy(best_inl, inl, (size_t)n); }
    }
    if (best_cnt < p->min_inliers) return 0;

    /* total least squares refit */
    double cx = 0, cy = 0;
    int m = 0;
    for (int q = 0; q < n; q++) if (best_inl[q]) { cx += px[q]; cy += py[q]; m++; }
    cx /= m; cy /= m;
    double sxx = 0, sxy = 0, syy = 0;
    for (int q = 0; q < n; q++) if (best_inl[q]) {
        double dx = px[q] - cx, dy = py[q] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    double ang = 0.5 * atan2(2 * sxy, sxx - syy);            /* principal direction */
    double tx = cos(ang), ty = sin(ang);
    double nx = -ty, ny = tx;
    if (nx * cx + ny * cy < 0) { nx = -nx; ny = -ny; }       /* normal points away from the robot */
    if (fabs(pgrd_wrap(atan2(ny, nx) - prior.th)) > p->max_angle) return 0;
    tx = ny; ty = -nx;
    int k = 0;
    for (int q = 0; q < n; q++) if (best_inl[q]) s[k++] = (px[q] - cx) * tx + (py[q] - cy) * ty;
    qsort(s, (size_t)k, sizeof(double), cmp_d);

    int pick = -1;
    double best_d = 1e18;
    if (nbeams <= 0) {                                        /* without beams: the largest gap, if door-sized */
        for (int q = 0; q + 1 < k; q++)
            if (pick < 0 || s[q + 1] - s[q] > s[pick + 1] - s[pick]) pick = q;
        if (pick < 0 || !(s[pick + 1] - s[pick] > p->wmin && s[pick + 1] - s[pick] < p->wmax)) return 0;
    }
    for (int q = 0; q + 1 < k && nbeams > 0; q++) {
        double g = s[q + 1] - s[q];
        if (g <= p->wmin || g >= p->wmax) continue;
        double mx = cx + tx * (s[q] + s[q + 1]) / 2, my = cy + ty * (s[q] + s[q + 1]) / 2;
        int free_ = 1;                                        /* beams through the middle half must pass the wall */
        for (int f = -1; f <= 1 && free_; f++) {
            double qx = mx + tx * 0.25 * f * g, qy = my + ty * 0.25 * f * g;
            double dist = hypot(qx - ox, qy - oy);
            double r = beam_range(beam_ang, beam_rng, nbeams, atan2(qy - oy, qx - ox));
            if (!(r > dist + p->free_margin)) free_ = 0;
        }
        if (!free_) continue;
        double d = hypot(mx + nx * p->wall_t / 2 - prior.x, my + ny * p->wall_t / 2 - prior.y);
        if (d < best_d) { best_d = d; pick = q; }
    }
    if (pick < 0) return 0;
    *width = s[pick + 1] - s[pick];
    door->x = cx + tx * (s[pick] + s[pick + 1]) / 2 + nx * p->wall_t / 2;
    door->y = cy + ty * (s[pick] + s[pick + 1]) / 2 + ny * p->wall_t / 2;
    door->th = atan2(ny, nx);
    return 1;
}

/* ------------------------------------------------------------------ tracker */
void pgrd_tracker_reset(pgrd_tracker *t) { memset(t, 0, sizeof(*t)); }

void pgrd_tracker_add(pgrd_tracker *t, pgrd_pose robot_odom, pgrd_pose door_in_robot) {
    pgrd_pose meas = pgrd_compose(robot_odom, door_in_robot);
    if (t->n_det == 0) {
        t->door_odom = meas;
    } else {
        double kf = 1.0 / (t->n_det + 1 < 20 ? t->n_det + 1 : 20);
        pgrd_pose d = pgrd_compose(pgrd_inverse(t->door_odom), meas);
        d.x *= kf; d.y *= kf; d.th *= kf;
        t->door_odom = pgrd_compose(t->door_odom, d);
    }
    t->n_det++;
}
