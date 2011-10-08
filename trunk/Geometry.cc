#include "Geometry.h"
#include <cmath>

double zeroOut(double numToZero) {
    if (fabs(numToZero) < 1e-8) {
        return 0.0;
    } else {
        return numToZero;
    }
}

double normalizeAngle(double angle) {
    while (angle < 0) {
        angle = angle + 2.0 * M_PI;
    }
    while (angle >= 2.0 * M_PI) {
        angle = angle - 2.0 * M_PI;
    }
    return angle;
}

void findIntersection(double theta1, double x1, double y1, double theta2, double x2, double y2, double results[]) {
    //This function determines the intersection point of a line with angle
    //theta2 with a line which is perpendicular to theta1.  tan is used to calculate
    //slopes of lines.  The points x1, y1, x2, and y2 are sufficient to define
    //line equations, allowing for intersection points to be determined.

    //Initialize results
    for (int i = 0; i < 3; i++) {
        results[i] = 0.0;
    }
    //Get an angle perpendicular to theta1
    double angle1 = theta1 + M_PI / 2.0;
    //Normalize both angles to [0,2*M_PI)
    angle1 = normalizeAngle(angle1);
    theta2 = normalizeAngle(theta2);

    //Calculate line variables (for readability only)
    double slope1 = zeroOut(tan(angle1));
    double intercept1 = zeroOut(y1 - slope1 * x1);
    double slope2 = zeroOut(tan(theta2));
    double intercept2 = zeroOut(y2 - slope2 * x2);

    //Check to see if angle1 is horizontal
    if (angle1 == 0 || angle1 == M_PI) {
        //If other angle is horizontal
        if (theta2 == 0 || theta2 == M_PI) {
            //Mark results invalid (never intersects)
            results[0] = -1;
            //Check if other angle is vertical
        } else if (theta2 == M_PI / 2.0 || theta2 == 3 * M_PI / 2.0) {
            //Results trival (vertical x value, horizontal y value)
            results[1] = x2;
            results[2] = y1;
            //Other angle is arbitrary
        } else {
            //Calculate X from line equation
            results[1] = (y1 - intercept2) / slope2;
            //Y is horizontal value
            results[2] = y1;
        }
        //Check to see if angle1 is vertial
    } else if (angle1 == M_PI / 2.0 || angle1 == 3.0 * M_PI / 2.0) {
        //If other angle is vertical
        if (theta2 == M_PI / 2.0 || theta2 == 3.0 * M_PI / 2.0) {
            //Mark results invalid (never intersects)
            results[0] = -1;
            //If other angle is horizontal
        } else if (theta2 == 0 || theta2 == M_PI) {
            //Results trivial (vertical x value, horizontal y value)
            results[1] = x1;
            results[2] = y2;
            //Other angle is arbitrary
        } else {
            //X is vertical line
            results[1] = x1;
            //Calculate Y from line equation
            results[2] = slope2 * x1 + intercept2;
        }
        //Both angles are arbitrary
    } else {
        //If the angles are the same (within reason)
        if (fabs(angle1 - theta2) < 0.000001) {
            //Mark as invalid (never intersects)
            results[0] = -1;
            //Angles good
        } else {
            //Calculate X value using both line equations
            results[1] = (intercept1 - intercept2) / (slope2 - slope1);
            //Calculate Y by plugging X into second equation
            results[2] = slope2 * results[1] + intercept2;
        }
    }
}

double angleToPoint(double startX, double startY, double endX, double endY) {
    //Get the numerator and denominator for tangent
    double numerator = zeroOut(endX - startX);
    double denominator = zeroOut(endY - startY);

    if (numerator == 0.0) {
        if (denominator == 0.0) {
            return 0.0;
        } else {
            if (denominator > 0) {
                return M_PI / 2.0;
            } else {
                return 3.0 * M_PI / 2.0;
            }
        }
    } else {
        if (denominator == 0.0) {
            if (numerator > 0) {
                return 0.0;
            } else {
                return M_PI;
            }
        } else {
            double tempVal = atan(numerator / denominator);
            if (numerator > 0) {
                if (denominator > 0) {
                    return M_PI/2.0 - tempVal;
                } else {
                    return fabs(tempVal) + (3 * M_PI / 2.0);
                }
            } else {
                if (denominator > 0) {
                    return fabs(tempVal) + M_PI/2.0;
                } else {
                    return ((M_PI/2.0) - fabs(tempVal)) + M_PI;
                }
            }
        }
    }
}

double directDistance(double x1, double y1, double x2, double y2) {
    double xDist = (x1 - x2);
    double yDist = (y1 - y2);
    return sqrt(xDist * xDist + yDist * yDist);
}

void translatePoint(double result[], double direction, double distance) {
    double changeX = 0.0;
    double changeY = 0.0;
    direction = normalizeAngle(direction);
    if (anglesEqual(direction, 0)) {
        changeX = distance;
    } else if (anglesEqual(direction, M_PI / 2.0)) {
        changeY = distance;
    } else if (anglesEqual(direction, M_PI)) {
        changeX = -distance;
    } else if (anglesEqual(direction, 3.0 * M_PI / 2.0)) {
        changeY = -distance;
    } else {
        changeX = distance * cos(direction);
        changeY = distance * sin(direction);
    }
    result[0] = result[0] + changeX;
    result[1] = result[1] + changeY;
}

bool anglesEqual(double angle1, double angle2) {
    return zeroOut(fabs(angle1 - angle2)) == 0.0;
}
