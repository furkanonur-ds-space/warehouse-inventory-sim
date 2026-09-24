/*
 * Print the route this layout gives, one waypoint a line.
 *
 * The format is not for people: it is what tests/compare_with_python.sh
 * diffs against the Python route, which is the only thing that says whether
 * this port is right.
 *
 *     ./route_tool ../scanner/layout.json
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "route.h"

/*
 * Where each camera ends up and how much shelf it sees from there.
 *
 * Printed because the waypoints do not carry it. The mount offsets and the
 * lens reach only the warning about a band of codes too tall to frame, and
 * in a warehouse whose bands all fit, a wrong offset changes nothing that
 * can be seen. It was found exactly that way: a centimetre of error passed
 * every check there was.
 */
static void print_limits(const route_waypoint_t *waypoints, int n)
{
	double last_x = 0.0;
	int i;

	for (i = 0; i < n; i++) {
		double hires_depth;
		double rear_depth;

		/* One line per lane, and a lane is a run of waypoints at the
		 * same x. */
		if (i > 0 && waypoints[i].x == last_x)
			continue;
		last_x = waypoints[i].x;

		hires_depth = waypoints[i].hires_depth - HIRES_MOUNT_X;
		printf("lane %.3f hires depth %.6f limit %.6f\n",
		       waypoints[i].x, hires_depth,
		       route_half_frame_m(HIRES_HFOV_DEG, HIRES_FRAME_W,
					  HIRES_FRAME_H, hires_depth));
		if (!waypoints[i].has_rear)
			continue;
		rear_depth = waypoints[i].rear_depth + REAR_MOUNT_X;
		printf("lane %.3f rear  depth %.6f limit %.6f\n",
		       waypoints[i].x, rear_depth,
		       route_half_frame_m(REAR_HFOV_DEG, REAR_FRAME_W,
					  REAR_FRAME_H, rear_depth));
	}
}

int main(int argc, char **argv)
{
	route_layout_t layout;
	route_waypoint_t waypoints[ROUTE_MAX_WAYPOINTS];
	const char *path;
	int limits = 0;
	int n = 0;
	int i;

	if (argc == 3 && strcmp(argv[1], "--limits") == 0) {
		limits = 1;
		path = argv[2];
	} else if (argc == 2) {
		path = argv[1];
	} else {
		fprintf(stderr, "usage: %s [--limits] <layout.json>\n", argv[0]);
		return 2;
	}
	if (route_load_layout(path, &layout) != 0)
		return 1;
	if (route_build(&layout, waypoints, ROUTE_MAX_WAYPOINTS, &n) != 0)
		return 1;

	if (limits) {
		print_limits(waypoints, n);
		return 0;
	}

	for (i = 0; i < n; i++) {
		printf("%.6f %.6f %.6f %.6f %.6f ", waypoints[i].x,
		       waypoints[i].y, waypoints[i].z, waypoints[i].yaw_deg,
		       waypoints[i].hires_depth);
		if (waypoints[i].has_rear)
			printf("%.6f\n", waypoints[i].rear_depth);
		else
			printf("none\n");
	}
	return 0;
}
