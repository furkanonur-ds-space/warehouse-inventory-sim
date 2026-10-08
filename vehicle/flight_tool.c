/*
 * Print the setpoints and yaw sweeps this vehicle would command.
 *
 * The format is not for people: it is what tests/compare_with_python.sh
 * diffs against the Python, which is the only thing that says whether this
 * port is right. Nine decimal places, because the point is to catch a
 * difference rather than to read the numbers.
 *
 *     ./flight_tool tests/legs.json
 */
#include <stdio.h>
#include <stdlib.h>

#include "flight.h"
#include "third_party/cJSON.h"

#define MAX_SETPOINTS 8192
#define MAX_YAW_STEPS 4096

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
		free(text);
		fclose(file);
		return NULL;
	}
	text[size] = '\0';
	fclose(file);
	return text;
}

static int point_from_json(const cJSON *array, flight_point_t *out)
{
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != 3)
		return -1;
	out->n = cJSON_GetArrayItem(array, 0)->valuedouble;
	out->e = cJSON_GetArrayItem(array, 1)->valuedouble;
	out->d = cJSON_GetArrayItem(array, 2)->valuedouble;
	return 0;
}

int main(int argc, char **argv)
{
	static flight_setpoint_t points[MAX_SETPOINTS];
	static double yaws[MAX_YAW_STEPS];
	char *text;
	cJSON *root;
	const cJSON *legs;
	const cJSON *turns;
	const cJSON *entry;
	int index;
	int rc = 1;

	if (argc != 2) {
		fprintf(stderr, "usage: %s <legs.json>\n", argv[0]);
		return 2;
	}
	text = read_whole_file(argv[1]);
	if (!text)
		return 1;
	root = cJSON_Parse(text);
	free(text);
	if (!root) {
		fprintf(stderr, "ERROR %s is not valid JSON\n", argv[1]);
		return 1;
	}

	legs = cJSON_GetObjectItemCaseSensitive(root, "legs");
	if (!cJSON_IsArray(legs)) {
		fprintf(stderr, "ERROR no legs in %s\n", argv[1]);
		goto done;
	}
	index = 0;
	cJSON_ArrayForEach(entry, legs) {
		flight_point_t start;
		flight_point_t target;
		flight_leg_t plan;
		int n = 0;
		int i;

		if (point_from_json(cJSON_GetObjectItemCaseSensitive(entry,
								     "start"),
				    &start) ||
		    point_from_json(cJSON_GetObjectItemCaseSensitive(entry,
								     "target"),
				    &target)) {
			fprintf(stderr, "ERROR leg %d has no start or target\n",
				index);
			goto done;
		}

		flight_leg_plan(start, target, &plan);
		printf("leg %d length %.9f climb %d speed %.9f max_time %.9f\n",
		       index, plan.length, plan.is_climb, plan.speed,
		       plan.max_time);

		if (flight_leg_setpoints(start, target, FLIGHT_SETPOINT_DT,
					 points, MAX_SETPOINTS, &n) != 0)
			goto done;
		printf("leg %d points %d\n", index, n);
		for (i = 0; i < n; i++)
			printf("leg %d point %d %.9f %.9f %.9f %.9f\n", index,
			       i, points[i].n, points[i].e, points[i].d,
			       points[i].fraction);
		index++;
	}

	turns = cJSON_GetObjectItemCaseSensitive(root, "turns");
	if (!cJSON_IsArray(turns)) {
		fprintf(stderr, "ERROR no turns in %s\n", argv[1]);
		goto done;
	}
	index = 0;
	cJSON_ArrayForEach(entry, turns) {
		double from = cJSON_GetObjectItemCaseSensitive(entry, "from")
				      ->valuedouble;
		double to = cJSON_GetObjectItemCaseSensitive(entry, "to")
				    ->valuedouble;
		int n = 0;
		int i;

		if (flight_heading_sweep(from, to, FLIGHT_SETPOINT_DT,
					 FLIGHT_YAW_SWEEP_RATE, yaws,
					 MAX_YAW_STEPS, &n) != 0)
			goto done;
		printf("turn %d steps %d\n", index, n);
		for (i = 0; i < n; i++)
			printf("turn %d step %d %.9f\n", index, i, yaws[i]);
		index++;
	}

	rc = 0;
done:
	cJSON_Delete(root);
	return rc;
}
