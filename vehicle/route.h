/*
 * The warehouse and the route through it, for the vehicle.
 *
 * A port of scanner/route.py, which stays where it is: it is the test bench
 * this is checked against, and the two must produce the same waypoints from
 * the same layout file.
 *
 * Nothing here talks to an autopilot, a camera or a pipe. It reads a layout
 * and answers where to fly.
 */
#ifndef ROUTE_H
#define ROUTE_H

#define ROUTE_MAX_FACES 64
#define ROUTE_MAX_LEVELS 16
#define ROUTE_MAX_NAME 32
#define ROUTE_MAX_WAYPOINTS 512

/* One scannable shelf face. code_low and code_high are the lowest and the
 * highest a code sits at on each level; the layout may carry a median
 * between them, which nothing uses. */
typedef struct {
	char name[ROUTE_MAX_NAME];
	double face_x;
	double yaw_deg;
	int has_code_z;
	int n_levels;
	double code_low[ROUTE_MAX_LEVELS];
	double code_high[ROUTE_MAX_LEVELS];
} route_face_t;

typedef struct {
	route_face_t faces[ROUTE_MAX_FACES];
	int n_faces;

	double flight_z[ROUTE_MAX_LEVELS];
	int n_levels;

	double y_south;
	double y_north;

	double hires_max_standoff;
	double rear_max_standoff;
	double shelf_standoff;
	double vehicle_half_span;
	double code_size_m;
	double code_plane_offset_m;
} route_layout_t;

/* rear_depth is only meaningful when has_rear is set: a face along a wall
 * has nothing behind it for the rear camera to read. */
typedef struct {
	double x;
	double y;
	double z;
	double yaw_deg;
	double hires_depth;
	double rear_depth;
	int has_rear;
} route_waypoint_t;

/* 0 on success, -1 with a message on stderr otherwise. */
int route_load_layout(const char *path, route_layout_t *layout);

/* Fills up to max waypoints and writes how many. 0 on success. */
int route_build(const route_layout_t *layout, route_waypoint_t *out, int max,
		int *n_out);

/* Where the codes on this face actually sit, which is not the shelf
 * surface: the label is on the box behind it. */
double route_code_plane_x(const route_layout_t *layout,
			  const route_face_t *face);

/* Half the camera's vertical field, in metres, at this distance. */
double route_half_frame_m(double hfov_deg, int frame_w, int frame_h,
			  double depth);

#endif
