/*
 * pgr_door - one-go narrow-door passing for wheeled robots (powered wheelchairs, AMRs), C99, libm only.
 * Chassis: differential drive, Ackermann (front steering), 4D4T (four-wheel drive, four-wheel independent steering).
 *
 * Controller: PGR (Path-Generating Regulator, 経路生成形レギュレータ; A. Takashima 2004, developed by the Hanajima
 *             Laboratory, Muroran Institute of Technology) wells in the DOOR frame.
 *   phi_n = sqrt(X^2 + n Y^2) around a well pose (X, Y in the well frame); the robot heading follows the flow
 *   direction atan2(-n Y, -X) (X < 0), whose streamlines Y = Y0 (X / X0)^n reach the well tangentially.
 *   Stage 0: well in front of the door at distance `pre`, so the whole body is aligned before its front reaches the
 *            wall: pre = xmax + wall_t / 2 + 5 v_near / k  (front extent + half wall + heading settling distance).
 *   Stage 1: well beyond the door (exit_dist); slow speed near the door.
 *   The law is pure state feedback: a disturbed robot (caster swing, wheel slip, localization jumps) is pulled back
 *   towards the streamline from wherever it is, without re-planning.
 *
 * Perception: the door is seen in a 2D scan (RANSAC wall line + gap + free-beam check) and fused in the odometry
 * frame, so the controller works on the robot pose RELATIVE TO THE DOOR (cm-level), not on the map pose.
 *
 * Frames: door frame = origin at the door centre on the wall mid-plane, x through the door (robot comes from x < 0),
 * y along the wall. Poses are (x, y, theta) of the reference point (see pgrd_ctrl_step_body).
 *
 * License: Apache-2.0
 */
#ifndef PGR_DOOR_H
#define PGR_DOOR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x, y, th;
} pgrd_pose;

/* ------------------------------------------------------------------ SE(2) helpers */
pgrd_pose pgrd_compose(pgrd_pose a, pgrd_pose b); /* b expressed in a's frame -> outer frame */
pgrd_pose pgrd_inverse(pgrd_pose a);
double pgrd_wrap(double a);

/* ------------------------------------------------------------------ controller */
typedef struct {
    double xmin, xmax, hw;  /* footprint about the rotation centre: x in [xmin, xmax], |y| <= hw */
    double wall_t;          /* wall thickness at the door */
    double n;               /* PGR exponent (3) */
    double k;               /* heading gain (1/s) */
    double v_far, v_near;   /* cruise speed / speed near the door */
    double w_max;           /* yaw-rate limit */
    double pre;             /* pre-door well distance (m); <= 0: derived from the footprint */
    double exit_dist;       /* exit well beyond the wall (m) */
    double goal_tol;        /* done when the axle is this close to the exit well */
    double dt;              /* control period (s), used for the flow feed-forward */
    int chassis;            /* PGRD_DIFF, PGRD_ACKERMANN, PGRD_4WS */
    double r_min;           /* Ackermann: smallest turning radius (m) */
    double beta_max;        /* 4WS: largest crab angle between heading and direction of motion (rad) */
} pgrd_params;

enum { PGRD_DIFF = 0, PGRD_ACKERMANN = 1, PGRD_4WS = 2 };

void pgrd_default_params(pgrd_params *p);   /* wheelchair: x in [-0.25, 0.80], hw 0.33, wall 0.12, differential */
/* defaults for a chassis type (n = 2 for Ackermann: lower peak curvature of the streamlines) */
void pgrd_default_params_for(pgrd_params *p, int chassis);
double pgrd_derived_pre(const pgrd_params *p);

typedef struct {
    pgrd_params p;
    double pre;
    int stage; /* 0: approach well, 1: exit well, 2: done */
} pgrd_ctrl;

enum { PGRD_APPROACH = 0, PGRD_EXIT = 1, PGRD_DONE = 2 };

void pgrd_ctrl_init(pgrd_ctrl *c, const pgrd_params *p);
/* pose: reference point in the door frame (drive axle for differential / Ackermann, body centre for 4WS);
 * v_meas: measured speed. Body-frame command (vx, vy, wz); vy != 0 only for 4WS (crab). Returns the stage.
 *   differential: heading follows the flow;  Ackermann: same, |wz| <= v / r_min;
 *   4WS: the DIRECTION OF MOTION follows the flow (crab angle <= beta_max) while the heading is turned to the door
 *        normal, so lateral offsets are removed by translating, not by swinging the body. */
int pgrd_ctrl_step_body(pgrd_ctrl *c, pgrd_pose pose, double v_meas, double *vx, double *vy, double *wz);
/* differential-drive form: (v, w) */
int pgrd_ctrl_step(pgrd_ctrl *c, pgrd_pose pose, double v_meas, double *v_cmd, double *w_cmd);
/* flow heading of phi_n for the well at (x, y) */
double pgrd_flow_heading(double x, double y, pgrd_pose well, double n);

/* ------------------------------------------------------------------ door detection (2D scan) */
typedef struct {
    double win;          /* points within this radius of the predicted door centre are used (m) */
    double thr;          /* RANSAC inlier distance (m) */
    int iters;           /* RANSAC iterations */
    int min_inliers;
    double wmin, wmax;   /* accepted gap width (m) */
    double wall_t;       /* wall thickness: the door centre is half a wall behind the near face */
    double max_angle;    /* max deviation of the wall normal from the prior (rad) */
    double free_margin;  /* a gap is free if beams through it reach this far beyond the wall line (m) */
} pgrd_detect_params;

void pgrd_detect_default(pgrd_detect_params *p);

/* pts: scan points in the ROBOT frame (x0, y0, x1, y1, ...); prior: predicted door pose in the robot frame
 * (x, y, normal heading). beam_ang / beam_rng (optional, nbeams > 0): the scan in the SENSOR frame (non-returns =
 * +inf) with the sensor at (ox, oy) in the robot frame; a gap then counts only if beams through it pass the wall
 * (the shadow of an object in front of the wall is rejected) and the free gap closest to the prior is taken.
 * rng_state: caller-owned RANSAC seed. Returns 1 and the door pose / clear width if a door was found. */
int pgrd_detect(const pgrd_detect_params *p, const double *pts, int npts, pgrd_pose prior,
                const double *beam_ang, const double *beam_rng, int nbeams, double ox, double oy,
                unsigned *rng_state, pgrd_pose *door, double *width);

/* ------------------------------------------------------------------ door tracking in the odometry frame */
typedef struct {
    int n_det;
    pgrd_pose door_odom;   /* fused door pose in the odometry frame (valid when n_det > 0) */
} pgrd_tracker;

void pgrd_tracker_reset(pgrd_tracker *t);
/* fuse one detection (door in the robot frame) taken at robot pose robot_odom; running mean over <= 20 detections */
void pgrd_tracker_add(pgrd_tracker *t, pgrd_pose robot_odom, pgrd_pose door_in_robot);

#ifdef __cplusplus
}
#endif
#endif
