#include <TrigDefinitions.h>


double euc_dist(const Point& p1, const Point& p2);
double triangleArea(const Triangle& triangle);
bool colinearOnSegment(const Edge& segment, const Point& p);
Circle circumscribedCircle(const Triangle& triangle);
int pointsOrientation(const Point& p1, const Point& p2, const Point& p3);
bool checkAngleBetween(double a1, double b, double a2);
bool segmentsDoIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2);