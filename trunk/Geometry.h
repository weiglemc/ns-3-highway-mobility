#ifndef INTERSECTION_GEOMETRY 
#define INTERSECTION_GEOMETRY 

double normalizeAngle(double angle);
void findIntersection(double theta1, double x1, double y1, double theta2, double x2, double y2, double results[]);
double angleToPoint(double startX, double startY, double endX, double endY);
double directDistance(double x1, double y1, double x2, double y2);
void translatePoint(double result[], double direction, double distance);
double zeroOut(double numToZero);
bool anglesEqual(double angle1, double angle2);
#endif
