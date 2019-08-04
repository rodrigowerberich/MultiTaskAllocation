#ifndef QUICK_BINARY_TREE_H_
#define QUICK_BINARY_TREE_H_

#include <vector>
#include <memory>

class NodeData{
public:
    NodeData(double key=0.0, int data=0):key{key}, data{data}{}
    bool operator<(const NodeData &right) const;
    bool operator>(const NodeData &right) const;
    double key;
    int data;
};

class Node{
public:
    Node(NodeData data);
    void insert(NodeData data);
    void printTree();
    int sortedUntil(int k, std::vector<NodeData>& sorted);
private:
    NodeData data;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
};

#endif //QUICK_BINARY_TREE_H_