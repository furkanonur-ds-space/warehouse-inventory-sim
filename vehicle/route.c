/*
 * A port of scanner/route.py. The reasoning behind every number is in that
 * file and in the commits that put it there; it is not repeated here, and
 * where the two differ the Python is right until a flight says otherwise.
 *
 * What is repeated here is anything the port itself can get wrong.
 */
#include "route.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "third_party/cJSON.h"

/* Room to leave between a propeller tip and a shelf. */
#define AISLE_CLEARANCE_M 0.05

/* M_PI is not in the standard this is built against, and pulling it in by
 * relaxing the standard would be a larger change than writing it down. */
#define ROUTE_PI 3.14159265358979323846

/*
 * Python's round() goes to the nearest value and settles a tie on the even
 * digit. C's round() settles a tie away from zero, so round(0.0005, 3)
 * would part company with Python on exactly the values a layout is most
 * likely to hold. nearbyint is in the mode C starts in, which is the same
 * nearest-even rule.
 */
static double round_to(double value, int places)
{
	double scale = pow(10.0, places);
	return nearbyint(value * scale) / scale;
}

static double json_number(const cJSON *parent, const char *key, double fallback)
{
	const cJSON *item = cJSON_GetObjectItemCaseSensitive(parent, key);
	if (!cJSON_IsNumber(item))
		return fallback;
	return item->valuedouble;
}

double route_half_frame_m(double hfov_deg, int frame_w, int frame_h,
			  double depth)
{
	double tan_half_v = tan(hfov_deg * ROUTE_PI / 180.0 / 2.0) *
			    (double)frame_h / (double)frame_w;
	return depth * tan_half_v * USABLE_FRAME;
}

double route_code_plane_x(const route_layout_t *layout,
			  const route_face_t *face)
{
	return face->face_x + (face->yaw_deg > 0 ? layout->code_plane_offset_m
						 : -layout->code_plane_offset_m);
}

static char *read_whole_file(const char *path)
{
	FILE *file = fopen(path, "rb");
	long size;
	char *text;

	if (!file) {
		fprintf(stderr, "ERROR cannot open %s\n", path);
		return NULL;
	}
	fseek(file, 0, SEEK_END);
	size = ftell(file);
	fseek(file, 0, SEEK_SET);
	if (size < 0) {
		fclose(file);
		return NULL;
	}
	text = malloc((size_t)size + 1);
	if (!text) {
		fclose(file);
		return NULL;
	}
	if (fread(text, 1, (size_t)size, file) != (size_t)size) {
		fprintf(stderr, "ERROR short read on %s\n", path);
		free(text);
		fclose(file);
		return NULL;
	}
	text[size] = '\0';
	fclose(file);
	return text;
}

int route_load_layout(const char *path, route_layout_t *layout)
{
	char *text = read_whole_file(path);
	cJSON *root;
	const cJSON *faces;
	const cJSON *flight_z;
	const cJSON *entry;
	int rc = -1;

	if (!text)
		return -1;
	root = cJSON_Parse(text);
	free(text);
	if (!root) {
		fprintf(stderr, "ERROR %s is not valid JSON\n", path);
		return -1;
	}

	memset(layout, 0, sizeof(*layout));

	flight_z = cJSON_GetObjectItemCaseSensitive(root, "flight_z");
	if (!cJSON_IsArray(flight_z)) {
		fprintf(stderr, "ERROR %s has no flight_z\n", path);
		goto done;
	}
	cJSON_ArrayForEach(entry, flight_z) {
		if (layout->n_levels >= ROUTE_MAX_LEVELS) {
			fprintf(stderr, "ERROR too many shelf levels\n");
			goto done;
		}
		layout->flight_z[layout->n_levels++] = entry->valuedouble;
	}

	layout->y_south = json_number(root, "y_south", 0.0);
	layout->y_north = json_number(root, "y_north", 0.0);
	layout->hires_max_standoff = json_number(root, "hires_max_standoff", 1.30);
	layout->rear_max_standoff = json_number(root, "rear_max_standoff", 1.10);
	layout->shelf_standoff = json_number(root, "shelf_standoff",
					     layout->hires_max_standoff);
	layout->vehicle_half_span = json_number(root, "vehicle_half_span", 0.0);
	layout->code_size_m = json_number(root, "code_size_m", 0.072);
	layout->code_plane_offset_m = json_number(root, "code_plane_offset_m", 0.0);

	faces = cJSON_GetObjectItemCaseSensitive(root, "aisle_faces");
	if (!cJSON_IsArray(faces)) {
		fprintf(stderr, "ERROR %s has no aisle_faces\n", path);
		goto done;
	}
	cJSON_ArrayForEach(entry, faces) {
		route_face_t *face;
		const cJSON *name;
		const cJSON *code_z;
		const cJSON *band;

		if (layout->n_faces >= ROUTE_MAX_FACES) {
			fprintf(stderr, "ERROR too many faces\n");
			goto done;
		}
		face = &layout->faces[layout->n_faces++];
		memset(face, 0, sizeof(*face));

		name = cJSON_GetObjectItemCaseSensitive(entry, "name");
		if (cJSON_IsString(name) && name->valuestring)
			snprintf(face->name, sizeof(face->name), "%s",
				 name->valuestring);
		face->face_x = json_number(entry, "face_x", 0.0);
		face->yaw_deg = json_number(entry, "yaw_deg", 0.0);

		code_z = cJSON_GetObjectItemCaseSensitive(entry, "code_z");
		if (!cJSON_IsArray(code_z))
			continue;
		face->has_code_z = 1;
		cJSON_ArrayForEach(band, code_z) {
			const cJSON *first;
			const cJSON *last;
			int count;

			if (face->n_levels >= ROUTE_MAX_LEVELS) {
				fprintf(stderr, "ERROR too many code bands\n");
				goto done;
			}
			if (!cJSON_IsArray(band))
				continue;
			count = cJSON_GetArraySize(band);
			if (count < 1)
				continue;
			/* The lowest and the highest. A layout may carry a
			 * median between them; nothing reads it. */
			first = cJSON_GetArrayItem(band, 0);
			last = cJSON_GetArrayItem(band, count - 1);
			face->code_low[face->n_levels] = first->valuedouble;
			face->code_high[face->n_levels] = last->valuedouble;
			face->n_levels++;
		}
	}

	rc = 0;
done:
	cJSON_Delete(root);
	return rc;
}

/* Where the vehicle flies to read a face: standoff out from the shelf
 * surface, on the side the camera looks from. */
static double face_lane_x(const route_face_t *face, double standoff)
{
	if (face->yaw_deg > 0)
		return face->face_x - standoff;
	return face->face_x + standoff;
}

/* The face across the aisle, or NULL for a row along a wall. */
static const route_face_t *facing_face(const route_layout_t *layout,
				       const route_face_t *face)
{
	double ahead = face->yaw_deg < 0 ? 1.0 : -1.0;
	const route_face_t *best = NULL;
	double best_gap = 0.0;
	int i;

	for (i = 0; i < layout->n_faces; i++) {
		const route_face_t *other = &layout->faces[i];
		double gap;

		if (other == face)
			continue;
		if (other->yaw_deg * face->yaw_deg >= 0)
			continue;
		if ((other->face_x - face->face_x) * ahead <= 0)
			continue;
		gap = fabs(other->face_x - face->face_x);
		if (!best || gap < best_gap) {
			best = other;
			best_gap = gap;
		}
	}
	return best;
}

/* How far the lane sits from each face of an aisle of this width, or -1
 * when one lane cannot read both. */
static int split_aisle(const route_layout_t *layout, double width,
		       double *hires_out, double *rear_out)
{
	double total = layout->hires_max_standoff + layout->rear_max_standoff;
	double hires = width * layout->hires_max_standoff / total;
	double rear = width - hires;

	if (rear > layout->rear_max_standoff) {
		rear = layout->rear_max_standoff;
		hires = width - rear;
	}
	if (hires > layout->hires_max_standoff)
		return -1;
	*hires_out = hires;
	*rear_out = rear;
	return 0;
}

static int aisle_fits(const route_layout_t *layout, double width)
{
	if (layout->vehicle_half_span <= 0)
		return 1;
	return width / 2.0 - layout->vehicle_half_span >= AISLE_CLEARANCE_M;
}

/* What a lane reads: one entry per face, with the camera that reads it and
 * how far that camera is from it. */
typedef struct {
	const route_face_t *face;
	double hfov_deg;
	int frame_w;
	int frame_h;
	double depth;
} lane_read_t;

typedef struct {
	double x;
	double yaw_deg;
	double hires_depth;
	double rear_depth;
	int has_rear;
	const route_face_t *hires_face;
	const route_face_t *rear_face;
	double z[ROUTE_MAX_LEVELS];
} lane_t;

/*
 * What altitude to fly at each level on this lane: the middle of the band of
 * code heights the lane has to read, and a warning when that band cannot fit
 * the frame whatever the altitude.
 */
static void lane_levels(const route_layout_t *layout, const lane_read_t *reads,
			int n_reads, double *levels)
{
	int index;

	for (index = 0; index < layout->n_levels; index++) {
		double low = 0.0;
		double high = 0.0;
		int with_bands = 0;
		int i;

		for (i = 0; i < n_reads; i++) {
			const route_face_t *face = reads[i].face;

			if (!face->has_code_z || index >= face->n_levels)
				continue;
			if (!with_bands || face->code_low[index] < low)
				low = face->code_low[index];
			if (!with_bands || face->code_high[index] > high)
				high = face->code_high[index];
			with_bands++;
		}

		if (with_bands != n_reads) {
			/* A layout that does not say where its codes are
			 * keeps the old behaviour. Guessing would be worse
			 * than the median it replaces. */
			levels[index] = layout->flight_z[index];
			continue;
		}

		levels[index] = round_to((low + high) / 2.0, 3);

		for (i = 0; i < n_reads; i++) {
			const route_face_t *face = reads[i].face;
			double axis = levels[index];
			double reach;
			double limit;

			reach = fmax(fabs(face->code_low[index] - axis),
				     fabs(face->code_high[index] - axis)) +
				layout->code_size_m / 2.0;
			limit = route_half_frame_m(reads[i].hfov_deg,
						   reads[i].frame_w,
						   reads[i].frame_h,
						   reads[i].depth);
			if (reach > limit)
				fprintf(stderr,
					"[WARN] face %s level %d: its codes "
					"span %.3f m, and the camera sees "
					"%.3f m of shelf from %.3f m away. "
					"%.3f m of that band falls outside "
					"the frame whatever the altitude, so "
					"this level cannot be read completely "
					"in one pass.\n",
					face->name, index + 1,
					face->code_high[index] -
						face->code_low[index],
					2 * limit, reads[i].depth,
					reach - limit);
		}
	}
}

int route_build(const route_layout_t *layout, route_waypoint_t *out, int max,
		int *n_out)
{
	lane_t lanes[ROUTE_MAX_FACES];
	int n_lanes = 0;
	const route_face_t *covered[ROUTE_MAX_FACES];
	int n_covered = 0;
	int heading_north = 1;
	int levels_ascending = 1;
	int i;
	int count = 0;

	for (i = 0; i < layout->n_faces; i++) {
		const route_face_t *face = &layout->faces[i];
		const route_face_t *opposite;
		double width;
		double hires;
		double rear;
		int j;
		int already = 0;

		for (j = 0; j < n_covered; j++)
			if (covered[j] == face)
				already = 1;
		if (already)
			continue;

		opposite = facing_face(layout, face);

		if (!opposite) {
			/* A row along a wall. The hires reads it alone, from
			 * as far back as it can still read. */
			hires = fmin(layout->shelf_standoff,
				     layout->hires_max_standoff);
			lanes[n_lanes].x = round_to(face_lane_x(face, hires), 3);
			lanes[n_lanes].yaw_deg = face->yaw_deg;
			lanes[n_lanes].hires_depth = hires;
			lanes[n_lanes].has_rear = 0;
			lanes[n_lanes].rear_depth = 0.0;
			lanes[n_lanes].hires_face = face;
			lanes[n_lanes].rear_face = NULL;
			n_lanes++;
			covered[n_covered++] = face;
			continue;
		}

		width = fabs(opposite->face_x - face->face_x);
		if (!aisle_fits(layout, width)) {
			fprintf(stderr,
				"[WARN] aisle between %s and %s is %.2f m; "
				"the vehicle is %.2f m across and needs "
				"%.2f m a side. Not flown.\n",
				face->name, opposite->name, width,
				2 * layout->vehicle_half_span,
				AISLE_CLEARANCE_M);
			covered[n_covered++] = face;
			covered[n_covered++] = opposite;
			continue;
		}

		if (split_aisle(layout, width, &hires, &rear) != 0) {
			/* Wider than both cameras together can cover, so
			 * each face gets its own pass. */
			const route_face_t *pair[2] = { face, opposite };

			for (j = 0; j < 2; j++) {
				double own = fmin(layout->shelf_standoff,
						  layout->hires_max_standoff);

				lanes[n_lanes].x =
					round_to(face_lane_x(pair[j], own), 3);
				lanes[n_lanes].yaw_deg = pair[j]->yaw_deg;
				lanes[n_lanes].hires_depth = own;
				lanes[n_lanes].has_rear = 0;
				lanes[n_lanes].rear_depth = 0.0;
				lanes[n_lanes].hires_face = pair[j];
				lanes[n_lanes].rear_face = NULL;
				n_lanes++;
			}
			covered[n_covered++] = face;
			covered[n_covered++] = opposite;
			continue;
		}

		lanes[n_lanes].x = round_to(face_lane_x(face, hires), 3);
		lanes[n_lanes].yaw_deg = face->yaw_deg;
		lanes[n_lanes].hires_depth = hires;
		lanes[n_lanes].rear_depth = rear;
		lanes[n_lanes].has_rear = 1;
		lanes[n_lanes].hires_face = face;
		lanes[n_lanes].rear_face = opposite;
		n_lanes++;
		covered[n_covered++] = face;
		covered[n_covered++] = opposite;
	}

	/* Where the optical axis goes on each lane. Done once the lane knows
	 * both the faces it reads and how far its cameras are from them. */
	for (i = 0; i < n_lanes; i++) {
		lane_read_t reads[2];
		int n_reads = 0;

		reads[n_reads].face = lanes[i].hires_face;
		reads[n_reads].hfov_deg = HIRES_HFOV_DEG;
		reads[n_reads].frame_w = HIRES_FRAME_W;
		reads[n_reads].frame_h = HIRES_FRAME_H;
		reads[n_reads].depth = lanes[i].hires_depth - HIRES_MOUNT_X;
		n_reads++;

		if (lanes[i].rear_face && lanes[i].has_rear) {
			reads[n_reads].face = lanes[i].rear_face;
			reads[n_reads].hfov_deg = REAR_HFOV_DEG;
			reads[n_reads].frame_w = REAR_FRAME_W;
			reads[n_reads].frame_h = REAR_FRAME_H;
			reads[n_reads].depth = lanes[i].rear_depth + REAR_MOUNT_X;
			n_reads++;
		}

		lane_levels(layout, reads, n_reads, lanes[i].z);

		/* Say when a lane flies somewhere other than the layout's own
		 * altitudes. Printed in Python's wording and number format on
		 * purpose: the port is checked by diffing what the two say as
		 * well as where they fly, and a lane that chose its altitude
		 * for a different reason would otherwise pass unnoticed. */
		{
			int differs = 0;
			int level;

			for (level = 0; level < layout->n_levels; level++)
				if (lanes[i].z[level] != layout->flight_z[level])
					differs = 1;
			if (differs) {
				fprintf(stderr, "[INFO] lane %s",
					lanes[i].hires_face->name);
				if (lanes[i].rear_face)
					fprintf(stderr, "/%s",
						lanes[i].rear_face->name);
				fprintf(stderr, ": flying [");
				for (level = 0; level < layout->n_levels; level++)
					fprintf(stderr, "%s%g",
						level ? ", " : "",
						lanes[i].z[level]);
				fprintf(stderr, "] rather than [");
				for (level = 0; level < layout->n_levels; level++)
					fprintf(stderr, "%s%g",
						level ? ", " : "",
						layout->flight_z[level]);
				fprintf(stderr, "], to put the axis on the "
						"codes\n");
			}
		}
	}

	for (i = 0; i < n_lanes; i++) {
		int step;

		for (step = 0; step < layout->n_levels; step++) {
			int index = levels_ascending
					    ? step
					    : layout->n_levels - 1 - step;
			double z = lanes[i].z[index];
			double ends[2];
			int e;

			ends[0] = heading_north ? layout->y_south
						: layout->y_north;
			ends[1] = heading_north ? layout->y_north
						: layout->y_south;

			for (e = 0; e < 2; e++) {
				if (count >= max) {
					fprintf(stderr,
						"ERROR more than %d waypoints\n",
						max);
					return -1;
				}
				out[count].x = lanes[i].x;
				out[count].y = ends[e];
				out[count].z = z;
				out[count].yaw_deg = lanes[i].yaw_deg;
				out[count].hires_depth = lanes[i].hires_depth;
				out[count].rear_depth = lanes[i].rear_depth;
				out[count].has_rear = lanes[i].has_rear;
				count++;
			}
			heading_north = !heading_north;
		}
		levels_ascending = !levels_ascending;
	}

	*n_out = count;
	return 0;
}
