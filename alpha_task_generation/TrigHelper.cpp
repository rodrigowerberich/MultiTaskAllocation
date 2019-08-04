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