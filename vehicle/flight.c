/*
 * A port of scanner/flight.py. Where the two differ the Python is right
 * until a flight says otherwise, and tests/compare_with_python.sh is what
 * says whether they differ.
 */
#include "flight.h"

#include <math.h>
#include <stdio.h>

void flight_leg_plan(flight_point_t start, flight_point_t target,
		     flight_leg_t *out)
{
	double vertical_span = fabs(target.d - start.d);
	double horizontal_span = sqrt((target.n - start.n) * (target.n - start.n) +
				      (target.e - start.e) * (target.e - start.e));

	out->length = sqrt(horizontal_span * horizontal_span +
			   vertical_span * vertical_span);
	/* A leg that is mostly vertical is a level change. Deciding by which
	 * span is larger, rather than by whether the altitude changed at all,
	 * keeps a lane change that also steps up a level from crawling the
	 * whole way across the warehouse. */
	out->is_climb = vertical_span > horizontal_span;
	out->speed = out->is_climb ? FLIGHT_CLIMB_SPEED : FLIGHT_CRUISE_SPEED;
	out->max_time = out->length / out->speed + FLIGHT_TIMEOUT_MARGIN;
}

int flight_leg_setpoints(flight_point_t start, flight_point_t target,
			 double dt, flight_setpoint_t *out, int max,
			 int *n_out)
{
	flight_leg_t plan;
	double travelled = 0.0;
	int count = 0;

	flight_leg_plan(start, target, &plan);

	/* The setpoint advances on a timer rather than waiting for the
	 * vehicle to arrive: a tolerance-gated stepper produces stop-start
	 * motion, and the camera needs each box to cross a steady sequence
	 * of frames. */
	for (;;) {
		double fraction;

		travelled = fmin(plan.length, travelled + plan.speed * dt);
		fraction = plan.length == 0.0 ? 1.0 : travelled / plan.length;

		if (count >= max) {
			fprintf(stderr, "ERROR more than %d setpoints\n", max);
			return -1;
		}
		out[count].n = start.n + (target.n - start.n) * fraction;
		out[count].e = start.e + (target.e - start.e) * fraction;
		out[count].d = start.d + (target.d - start.d) * fraction;
		out[count].fraction = fraction;
		count++;

		if (travelled >= plan.length)
			break;
	}

	*n_out = count;
	return 0;
}

int flight_heading_sweep(double start_yaw, double target_yaw, double dt,
			 double rate, double *out, int max, int *n_out)
{
	double delta;
	double duration;
	int steps;
	int i;

	/*
	 * The short way round: -90 to +90 is 180 degrees whichever way it is
	 * written, but +170 to -170 is 20 and not 340.
	 *
	 * Python's % always returns a value with the sign of the divisor, and
	 * C's fmod keeps the sign of the dividend, so fmod alone parts
	 * company with the original on every turn that wraps. fmod twice
	 * with an addition in between is the Python rule.
	 */
	delta = fmod(fmod(target_yaw - start_yaw + 180.0, 360.0) + 360.0,
		     360.0) - 180.0;

	duration = fabs(delta) / rate;
	if (duration < 0.1)
		duration = 0.1;
	steps = (int)(duration / dt);

	if (steps > max) {
		fprintf(stderr, "ERROR more than %d yaw steps\n", max);
		return -1;
	}
	for (i = 0; i < steps; i++)
		out[i] = start_yaw + delta * ((double)(i + 1) / (double)steps);

	*n_out = steps;
	return 0;
}
