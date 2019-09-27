#include <TrigHelper.h>
#include <cmath>

using namespace std;

double euc_dist(const Point& p1, const Point& p2){
    auto d_vector = p1-p2;
    return sqrtf(Point::dotProduct(d_vector, d_vector));
}

double triangleArea(const Triangle& triangle){
    auto v1 = triangle[1]-triangle[0];
    auto v2 = triangle[2]-triangle[0];
    return fabs(Point::crossProduct(v1,v2)/2.0);
}

bool colinearOnSegment(const Edge& s, const Point& p){
    if( (p.x <= max(s[0].x, s[1].x)) && (p.x >= min(s[0].x, s[1].x)) && 
        (p.y <= max(s[0].y, s[1].y)) && (p.y >= min(s[0].y, s[1].y))){
            return true;
        }
    return false;
}

Circle circumscribedCircle(const Triangle& triangle){
    const Point& a = triangle[0];
    const Point& b  = triangle[1];
    const Point& c  = triangle[2];
    double a0s = pow(a.x, 2);
    double a1s = pow(a.y, 2);
    double a01s = (a0s+a1s);
    double b01s = pow(b.x,2)+pow(b.y,2);
    double c01s = pow(c.x,2)+pow(c.y,2);
    double alpha1 = a01s-b01s;
    double beta1 = a.x-b.x;
    double gamma1 = a.y-b.y;
    double alpha2 = b01s-c01s;
    double beta2 = b.x-c.x;
    double gamma2 = b.y-c.y;
    double E = (alpha2*beta1-alpha1*beta2)/(gamma1*beta2-gamma2*beta1);
    double D = (-E*gamma1-alpha1)/beta1;
    double F = -a0s-a1s-D*a.x-E*a.y;
    Circle circle = {
        {-D/2, -E/2},
        0
    };
    circle.radius = sqrt(pow(circle.center.x,2)+pow(circle.center.y,2)-F);
    return circle;
}

int pointsOrientation(const Point& p1, const Point& p2, const Point& p3){
    double val = (p2.y-p1.y)*(p3.x-p2.x) - (p2.x-p1.x)*(p3.y-p2.y);
    if (fabs(val) < 0.00001){
        return 0;
    }
    return (val > 0)?1:2;
}

bool checkAngleBetween(double a1, double b, double a2){
    if(a1 > a2){
        swap(a1, a2);
    }
    if(fabs(a2-a1) > M_PI){
        return ((b >= a2) || (b <= a1));
    }else{
        return ((a1 <= b) && (b <= a2));
    }
}

bool segmentsDoIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2){
    int o1 = pointsOrientation(p1, q1, p2);
    int o2 = pointsOrientation(p1, q1, q2);
    int o3 = pointsOrientation(p2, q2, p1);
    int o4 = pointsOrientation(p2, q2, q1);

    if((o1 != o2) && (o3 != o4)){
        return true;
    }

    if(o1 == 0 && colinearOnSegment({p1, q1}, p2)){
        return true;
    }

    if(o2 == 0 && colinearOnSegment({p1, q1}, q2)){
        return true;
    }

    if(o3 == 0 && colinearOnSegment({p2, q2}, p1)){
        return true;
    }

    if(o4 == 0 && colinearOnSegment({p2, q2}, q1)){
        return true;
    }

    return false;
}

double distancePointAndVector(const Point& p, const Edge& edge){
    // To analyze the distance from a point a segment
    // we create to vector a and b with the same origin edge[0];
    Point a = p - edge[0];
    Point b = edge[1] - edge[0];
    double b_absolute = b.absolute();
    // a1 is the projection of a over the b vector
    double a1 = Point::dotProduct(a,b)/(b_absolute*b_absolute);
    // If a1 is between 0 and 1, the point in the line segment is not of
    // of the extremities and we use the distance from a line formula
    if ( 0 < a1 && a1 < 1 ){
        // This calculates the height of p over the vector b,
        // which is the distance
        return std::abs(Point::crossProduct(a,b))/b_absolute;
    }else{
        double distance_0 = Point::euclideanDistance(p, edge[0]);
        double distance_1 = Point::euclideanDistance(p, edge[1]);
        return std::min(distance_0, distance_1);
    }
}

std::tuple<Point, double> calculateClosestPointInSegment(const Point& p, const Edge& edge){
    // To analyze the distance from a point a segment
    // we create to vector a and b with the same origin edge[0];
    Point a = p - edge[0];
    Point b = edge[1] - edge[0];
    double b_absolute = b.absolute();
    // a1 is the projection of a over the b vector
    double a1 = Point::dotProduct(a,b)/(b_absolute*b_absolute);
    // If a1 is between 0 and 1, the point in the line segment is not of
    // of the extremities and we use the distance from a line formula
    if ( 0 < a1 && a1 < 1 ){
        // This calculates the height of p over the vector b,
        // which is the distance
        double distance = std::abs(Point::crossProduct(a,b))/b_absolute;
        Point closest_point = edge[0]+b*a1;
        return std::make_tuple(closest_point, distance);
    }else{
        double distance_0 = Point::euclideanDistance(p, edge[0]);
        double distance_1 = Point::euclideanDistance(p, edge[1]);
        return ( distance_0 < distance_1 )?(std::make_tuple(edge[0], distance_0)):(std::make_tuple(edge[1], distance_1));
    }
}


struct LineRepresentation{
    double m;
    double b;
    bool parallel_to_y;
    double x_value;
};

LineRepresentation calculateLineRepresentation(const Point& p, const Point& q){
    double delta_x = (p.x - q.x);
    if ( std::fabs(delta_x) <= 2*std::numeric_limits<double>::min()){
        return LineRepresentation{0.0,0.0,true,p.x};
    }
    return LineRepresentation{
        (p.y-q.y)/delta_x,
        ((p.x*q.y)-(p.y*q.x))/delta_x,
        false,
        0.0        
    };
}

std::tuple<Point, bool> calculateIntersection(const Point& p1, const Point& q1, const Point& p2, const Point& q2){
    LineRepresentation l1 = calculateLineRepresentation(p1,q1);
    LineRepresentation l2 = calculateLineRepresentation(p2,q2);
    if(!l1.parallel_to_y && !l2.parallel_to_y){
        // No line is parallel to y axis, so both can be described by
        // y = m*x+b
        double delta_m = (l1.m -l2.m);
        if( std::fabs(delta_m) <= 2*std::numeric_limits<double>::min()){
            // If both m are equal, the lines are parallel
            return std::make_tuple(Point{0,0}, false);
        }
        // If not parallel, the following equations calculate the meeting point
        return std::make_tuple(
            Point{
                (l2.b-l1.b)/delta_m,
                ((l1.m*l2.b)-(l2.m*l1.b))/delta_m
            },
            true
        );
    }else if(l1.parallel_to_y && l2.parallel_to_y){
        // In this case both lines are parallel
        return std::make_tuple(Point{0,0}, false);
    }else{
        // We assume the first line is parallel to y-axis, if not, we 
        // not we swap them
        if(l2.parallel_to_y){
            std::swap(l1,l2);
        }
        // Since l1 is parallel to the y axis, its equation is
        //  x = c (c constant)
        // The meeting point with a line with equation
        //  y = m*x+b
        // will be the point
        // (c, m*c+b)
        return std::make_tuple(
            Point(
                l1.x_value, 
                l2.m*l1.x_value+l2.b
            ), 
            true
        );
    }
}
