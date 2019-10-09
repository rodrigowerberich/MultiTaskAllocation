#ifndef ROBOTSPOSITIONEDGECOLLISIONCHECKER_H__
#define ROBOTSPOSITIONEDGECOLLISIONCHECKER_H__

#include <ProblemRepresentation.h>
#include <TrigDefinitions.h>
#include <RobotsPosition.h>
#include <RobotsPositionCollisionCheckVertices.h>
#include <EdgeStorage.h>

class RobotsPositionEdgeCollisionChecker{
private:
    const ProblemRepresentation& m_problem_representation;
    double m_check_size;
    bool collisionPoint(const Point& p) const;
    bool hasVisibility(const Point& p1, const Point& p2) const;
    bool hasConnectionToRouter(const Point& p) const;
    bool checkPathCollisionAndMapConnection(const RobotsPosition& rp1, const RobotsPosition& rp2, std::vector<Points>& robots_path_points, std::vector<std::vector<bool>>& robots_path_connection, std::vector<bool>& robots_fully_connected, RobotPositionCollisionCheckVertices& collision_check_vertices, bool& is_any_robot_fully_connected, bool& are_all_robots_fully_connected) const;
    bool checkIfLastPositionIsInvalid(std::vector<Points>& robots_path_points, std::vector<std::vector<bool>>& robots_path_connection, RobotPositionCollisionCheckVertices& collision_check_vertices, EdgeStorage& visible_edges, EdgeStorage& not_visible_edges) const;
public:
    RobotsPositionEdgeCollisionChecker(const ProblemRepresentation& problem_representation, double check_size);
    bool operator()(const RobotsPosition& rp1, const RobotsPosition& rp2) const;
};

#endif //ROBOTSPOSITIONEDGECOLLISIONCHECKER_H__