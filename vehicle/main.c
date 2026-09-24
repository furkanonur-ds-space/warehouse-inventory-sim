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

#include "route.h"

int main(int argc, char **argv)
{
	route_layout_t layout;
	route_waypoint_t waypoints[ROUTE_MAX_WAYPOINTS];
	int n = 0;
	int i;

	if (argc != 2) {
		fprintf(stderr, "usage: %s <layout.json>\n", argv[0]);
		return 2;
	}
	if (route_load_layout(argv[1], &layout) != 0)
		return 1;
	if (route_build(&layout, waypoints, ROUTE_MAX_WAYPOINTS, &n) != 0)
		return 1;

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
