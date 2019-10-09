#include <RobotsPositionEdgeCollisionChecker.h>

// Private methods ----------------------------------------------------

bool RobotsPositionEdgeCollisionChecker::collisionPoint(const Point& p) const{
    return m_problem_representation.getObstructedArea()->containsPoint(p);
}

// Public methods -----------------------------------------------------