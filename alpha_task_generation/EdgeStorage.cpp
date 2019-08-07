#include <EdgeStorage.h>


EdgeI EdgeStorage::orderEdge(const EdgeI& edge) const{
    if(edge[0] < edge[1]){
        return {edge[0], edge[1]};
    }else{
        return {edge[1], edge[0]};
    }
}
bool EdgeStorage::containsEdge(const EdgeI& edge) const{
    EdgeI search_edge = orderEdge(edge);
    const auto& connected_points = m_storage.find(search_edge[0]);
    if(connected_points != m_storage.end()){
        return (connected_points->second.count(search_edge[1]) > 0);
    }
    return false;
}
void EdgeStorage::addEdge(const EdgeI& edge){
    EdgeI search_edge = orderEdge(edge);
    auto& connected_points = m_storage[search_edge[0]];
    connected_points.insert(search_edge[1]);
}
void EdgeStorage::addTriangle(const TriangleI& triangle){
    addEdge({triangle[0], triangle[1]});
    addEdge({triangle[0], triangle[2]});
    addEdge({triangle[1], triangle[2]});
}
void EdgeStorage::removeEdge(const EdgeI& edge){
    EdgeI search_edge = orderEdge(edge);
    auto connected_points = m_storage.find(search_edge[0]);
    if(connected_points != m_storage.end()){
        connected_points->second.erase(search_edge[1]);
        if(connected_points->second.empty()){
            m_storage.erase(search_edge[0]);
        }
    }
}
std::string EdgeStorage::toString(){
    std::string aux;
    for(const auto& connect_points: m_storage){
        for(const auto& second_point: connect_points.second){
            char buffer[50];
            snprintf(buffer, 50, "%d:%d|", connect_points.first, second_point);
            aux.append(buffer);
        }
    }
    return aux;
}

EdgesI EdgeStorage::toEdges(){
    EdgesI aux;
    for(const auto& connect_points: m_storage){
        for(const auto& second_point: connect_points.second){
            aux.push_back({connect_points.first, second_point});
        }
    }
    return aux;
}

