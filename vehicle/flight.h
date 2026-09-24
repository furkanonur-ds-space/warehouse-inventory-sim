/*
 * What to command, and when, to fly one leg of the route.
 *
 * A port of scanner/flight.py, checked against it by
 * tests/compare_with_python.sh. Only the arithmetic: nothing here knows
 * whether the vehicle arrived or what the markers say.
 */
#ifndef FLIGHT_H
#define FLIGHT_H

/* One speed in every aisle, fixed at 1 m/s by a decision above this work. */
#define FLIGHT_CRUISE_SPEED 1.0
#define FLIGHT_CLIMB_SPEED 0.15

#define FLIGHT_SETPOINT_DT 0.1
#define FLIGHT_TIMEOUT_MARGIN 20.0

/* A sudden large yaw setpoint makes the vehicle spin at its maximum rate,
 * which blurs the tracking cameras and destroys VIO. */
#define FLIGHT_YAW_SWEEP_RATE 30.0

typedef struct {
	double n;
	double e;
	double d;
} flight_point_t;

typedef struct {
	double length;
	int is_climb;
	double speed;
	double max_time;
} flight_leg_t;

typedef struct {
	double n;
	double e;
	double d;
	double fraction;
} flight_setpoint_t;

void flight_leg_plan(flight_point_t start, flight_point_t target,
		     flight_leg_t *out);

/* Fills up to max setpoints and writes how many. -1 if it would not fit. */
int flight_leg_setpoints(flight_point_t start, flight_point_t target,
			 double dt, flight_setpoint_t *out, int max,
			 int *n_out);

/* The yaw targets that carry the vehicle from one heading to another, the
 * short way round. */
int flight_heading_sweep(double start_yaw, double target_yaw, double dt,
			 double rate, double *out, int max, int *n_out);

#endif
