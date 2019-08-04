#include <TrigDefinitions.h>
#include <cmath>

using namespace std;

TriangleI PI2TI(const PointsI& points){
    TriangleI triangle = {0,0,0};
    int points_size = min(static_cast<int>(points.size()), 3);
    for(int i = 0; i < points_size; i++){
        triangle[i] = points[i];
    }
    return triangle;
}

std::ostream& operator<<(std::ostream& os, const Edge& e){
    return os << "(" << e[0] << ", " << e[1] << ")";
}


std::ostream& operator<<(std::ostream& os, const Edges& es){
    os << "[";
    if(es.size() > 0){
        size_t size = es.size()-1;
        for(size_t i=0; i<size; i++){
            os << es[i] << ", ";
        }
        os << es[size];
    }
    return os << "]";
}


std::ostream& operator<<(std::ostream& os, const Points& ps){
    os << "[";
    if(ps.size() > 0){
        size_t size = ps.size()-1;
        for(size_t i=0; i<size; i++){
            os << ps[i] << ", ";
        }
        os << ps[size];
    }
    return os << "]";
}


std::ostream& operator<<(std::ostream& os, const Triangle& t){
    return os << "[" << t[0] << ", " << t[1] << ", " << t[2] << "]"; 
}

std::ostream& operator<<(std::ostream& os, const Triangles& ts){
    os << "[";
    if(ts.size() > 0){
        size_t size = ts.size()-1;
        for(size_t i=0; i<size; i++){
            os << ts[i] << ", ";
        }
        os << ts[size];
    }
    return os << "]";
}

std::ostream& operator<<(std::ostream& os, const EdgeI& e){
    return os << "(" << e[0] << ", " << e[1] << ")";
}


std::ostream& operator<<(std::ostream& os, const EdgesI& es){
    os << "[";
    if(es.size() > 0){
        size_t size = es.size()-1;
        for(size_t i=0; i<size; i++){
            os << es[i] << ", ";
        }
        os << es[size];
    }
    return os << "]";
}


std::ostream& operator<<(std::ostream& os, const PointsI& ps){
    os << "[";
    if(ps.size() > 0){
        size_t size = ps.size()-1;
        for(size_t i=0; i<size; i++){
            os << ps[i] << ", ";
        }
        os << ps[size];
    }
    return os << "]";
}


std::ostream& operator<<(std::ostream& os, const TriangleI& t){
    return os << "[" << t[0] << ", " << t[1] << ", " << t[2] << "]"; 
}

std::ostream& operator<<(std::ostream& os, const TrianglesI& ts){
    os << "[";
    if(ts.size() > 0){
        size_t size = ts.size()-1;
        for(size_t i=0; i<size; i++){
            os << ts[i] << ", ";
        }
        os << ts[size];
    }
    return os << "]";
}

