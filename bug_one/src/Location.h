#ifndef _LOCATTION_H_
#define _LOCATION_H_

#include <math.h>

//a small struct to hold my position.
typedef struct Location {
    double x;
    double y;
    double bearing;
    double xy_offset;
	double br_offset;

    Location() : x(0), y(0), bearing(0), xy_offset(0.25), br_offset(0.05) {};
	Location(double nx, double ny, double nb) : x(nx), y(ny), bearing(nb), xy_offset(0.25), br_offset(0.2) {};
	bool isEqualAproxLoc(Location& current) {
		return ( findRange(current) < xy_offset );
    }

    bool isEqualAproxBearing(Location& current) {
		return ((bearing-2*br_offset < current.bearing) && (current.bearing < bearing+2*br_offset));
	}
	
	double findRange(Location& current) {
		return sqrt( pow(x-current.x,2) + pow(y-current.y,2) );
	}

} position_s;

#endif
