/*
 * APRSWorldMap.c
 *
 *  Created on: Jul 4, 2015
 *      Author: dongfang
 */
#include "Types.h"
#include "GPS.h"
#include "Globals.h"
#include "APRS.h"
#include "WorldMap.h"

static const APRSPolygonVertex_t boundary144390[] = BOUNDARY_144390;
static const APRSPolygonVertex_t boundary144620[] = BOUNDARY_144620;
static const APRSPolygonVertex_t boundary144640[] = BOUNDARY_144640;
static const APRSPolygonVertex_t boundary144660[] = BOUNDARY_144660;
static const APRSPolygonVertex_t boundary144800[] = BOUNDARY_144800;
static const APRSPolygonVertex_t boundary144930[] = BOUNDARY_144930;
static const APRSPolygonVertex_t boundary145010[] = BOUNDARY_145010;
static const APRSPolygonVertex_t boundary145175[] = BOUNDARY_145175;
static const APRSPolygonVertex_t boundary145525[] = BOUNDARY_145525;
static const APRSPolygonVertex_t boundary145575[] = BOUNDARY_145575;

static const APRSPolygonVertex_t core144390[] = CORE_144390;
static const APRSPolygonVertex_t core144620[] = CORE_144620;
static const APRSPolygonVertex_t core144640[] = CORE_144640;
static const APRSPolygonVertex_t core144660[] = CORE_144660;
static const APRSPolygonVertex_t core144800[] = CORE_144800;
// 144930 missing
// 145010 missing
static const APRSPolygonVertex_t core145175[] = CORE_145175; // CORE_145175 has no boundary
//145525 missing
static const APRSPolygonVertex_t core145575[] = CORE_145575;

const APRSFrequencyRegion_t APRS_WORLD_MAP[] = { { .frequency = 144390000,
		.boundary = boundary144390, .core = core144390 }, { .frequency = 144620000,
		.boundary = boundary144620, .core = core144620 }, { .frequency = 144640000,
		.boundary = boundary144640, .core = core144640 }, { .frequency = 144660000,
		.boundary = boundary144660, .core = core144660 }, { .frequency = 144800000,
		.boundary = boundary144800, .core = core144800 }, { .frequency = 144930000,
		.boundary = boundary144930, .core = 0 }, { .frequency = 145010000,
		.boundary = boundary145010, .core = 0 }, { .frequency = 145175000,
		.boundary = boundary145175, .core = core145175 }, { .frequency = 145525000,
		.boundary = boundary145525, .core = 0 }, { .frequency = 145575000,
		.boundary = boundary145575, .core = core145575 }, };

const uint8_t APRS_WORLD_MAP_LENGTH = sizeof(APRS_WORLD_MAP)
		/ sizeof(APRSFrequencyRegion_t);


static boolean checkWithinPolygon(int16_t lat, int16_t lon,
		const APRSPolygonVertex_t* boundaryList, uint16_t* index) {

	int16_t initialLat = boundaryList[*index].lat;
	int16_t initialLon = boundaryList[*index].lon;
	boolean result = true;
	// check for all segments AB in clockwise direction that angle (AB, AP) is positive
	do {
		int16_t previousLon = boundaryList[*index].lon;
		int16_t previousLat = boundaryList[*index].lat;
		// trace_printf("Trying: lat:%d, lon:%d @ index %d\n", previousLat, previousLon, *index);
		(*index)++;
		int ap0 = lon - previousLon;
		int ap1 = lat - previousLat;
		int ab0 = boundaryList[*index].lon - previousLon;
		int ab1 = boundaryList[*index].lat - previousLat;
		int cross = ap0 * ab1 - ap1 * ab0;
		if (cross < 0) {
			result = false;
		}
	} while (boundaryList[*index].lat != initialLat
			|| boundaryList[*index].lon != initialLon);
	// Now index points at the last vertex which is same values as the first..
	// Take it one step further.
	// trace_printf("Return to origin detected (or went stoopid)\n");
	(*index)++;
	return result;
}

static boolean checkWithinRegion(int16_t lat, int16_t lon,const APRSFrequencyRegion_t* region) {
	boolean within = false;
	uint16_t vertexIndex = 0;
	//trace_printf("Trying region: %u\n", region->frequency);
	while (!within
			&& region->boundary[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
		within = checkWithinPolygon(lat, lon, region->boundary, &vertexIndex);
	}
	return within;
}

static boolean checkWithinCore(int16_t lat, int16_t lon, const APRSFrequencyRegion_t* region) {
	boolean within = false;
	uint16_t vertexIndex = 0;
	while (!within
			&& region->core[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
		within = checkWithinPolygon(lat, lon, region->core, &vertexIndex);
	}
	return within;
}

static void APRS_frequencies(int16_t lat, int16_t lon, boolean* frequenciesVector, boolean* isCoreVector) {
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		boolean isWithinRegion = checkWithinRegion(lat, lon, APRS_WORLD_MAP+i);
		if (isWithinRegion) {
			frequenciesVector[i] = true;
			isCoreVector[i] = checkWithinCore(lat, lon, APRS_WORLD_MAP+i);
		} else {
			frequenciesVector[i] = false;
			isCoreVector[i] = false;
		}
	}
}

void APRS_frequenciesFromPosition(
		const Location_t* position,
		boolean* frequenciesVector,
		boolean* isCoreVector) {
	APRS_frequencies((int16_t)(lastNonzeroPosition.lat + 0.5),
			(int16_t)(lastNonzeroPosition.lon + 0.5), frequenciesVector, isCoreVector);
}


/**
 * All debugging stuff below!
 */
void checkWorldMap_Orientation_Polygon(const APRSPolygonVertex_t* boundaryList,
		uint16_t* index) {
	int16_t initialLat = boundaryList[*index].lat;
	int16_t initialLon = boundaryList[*index].lon;
	int16_t previousLonVector = 0;									// n-1 to n
	int16_t previousLatVector = 0;
	uint8_t ttl = 100;
	// check for all segments AB in clockwise direction that angle (AB, AP) is positive
	do {
		int16_t previousLon = boundaryList[*index].lon;
		int16_t previousLat = boundaryList[*index].lat;
		(*index)++;
		int16_t nextLonVector = boundaryList[*index].lon - previousLon;	// n to n+1
		int16_t nextLatVector = boundaryList[*index].lat - previousLat;
		int cross = previousLatVector * nextLonVector
				- previousLonVector * nextLatVector;

		if (cross < 0) {
			// trace_printf("Concave polygon near {%d,%d}\n", previousLon, previousLat);
		}

		previousLonVector = nextLonVector;
		previousLatVector = nextLatVector;
		if (--ttl == 0) {
			// trace_printf("Never ending loop near {%d,%d}\n", previousLon, previousLat);
		}
	} while (boundaryList[*index].lat != initialLat
			|| boundaryList[*index].lon != initialLon);
	// Now index points at the last vertex which is same values as the first..
	// Take it one step further.
	// trace_printf("Return to origin detected (or went stoopid)\n");
	(*index)++;
}

void APRS_checkWorldMapBoundaries_convex_polygons() {
	trace_printf("Boundary convex Check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t vertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		//trace_printf("Trying region: %u\n", region->frequency);
		while (region->boundary[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			checkWorldMap_Orientation_Polygon(region->boundary, &vertexIndex);
		}
	}
	trace_printf("Boundary convex Check done!\n");
}

void APRS_checkWorldMapCoreAreas_convex_polygons() {
	trace_printf("Core Area convex Check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t vertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		// trace_printf("Trying cores for frequency %u\n", region->frequency);
		if (region->core == 0)
			continue;
		while (region->core[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			// trace_printf("Trying poly starting at {%d,%d}\n",region->core[vertexIndex].lon,region->core[vertexIndex].lat);
			checkWorldMap_Orientation_Polygon(region->core, &vertexIndex);
		}
	}
	trace_printf("Core Area convex done!\n");
}

void APRS_checkCoreVertexExcursion() {
	trace_printf("Core area excursion check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t coreVertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		// trace_printf("Trying cores for frequency %u\n", region->frequency);
		if (region->core == 0)
			continue;
		while (region->core[coreVertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			// trace_printf("Trying poly starting at {%d,%d}\n",region->core[vertexIndex].lon,region->core[vertexIndex].lat);
			if (!checkWithinRegion(region->core[coreVertexIndex].lat, region->core[coreVertexIndex].lon, region)) {
				trace_printf("Excursing core vertex at {%d,%d}\n", region->core[coreVertexIndex].lon, region->core[coreVertexIndex].lat);
			}
			coreVertexIndex++;
		}
	}
	trace_printf("Core area excursion check done!\n");
}

void APRS_debugWorldMap() {
	boolean frequencyList[APRS_WORLD_MAP_LENGTH];
	boolean core[APRS_WORLD_MAP_LENGTH];

	for (int16_t lat = 70; lat >= -70; lat--) {
		for (int16_t lon = -179; lon <= 180; lon++) {
			APRS_frequencies(lat, lon, frequencyList, core);
			trace_printf("%d;%d;", lat, lon);
			boolean needsComma = false;
			for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (frequencyList[i]) {
					if (needsComma)
						trace_printf(",");
					else
						needsComma = true;
					trace_printf("%u", APRS_WORLD_MAP[i].frequency);
				}
			}
			trace_printf("\n");
		}
	}
}

void APRS_debugFrequency(boolean* result) {
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		trace_printf("Frequency %u:", APRS_WORLD_MAP[i].frequency);
		if (result[i]) {
			trace_printf("*");
		}
		trace_printf("\n");
	}
}
