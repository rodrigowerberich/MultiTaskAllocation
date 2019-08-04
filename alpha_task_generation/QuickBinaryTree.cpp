#include <QuickBinaryTree.h>
#include <iostream>

bool NodeData::operator<(const NodeData &right) const{
    return this->key < right.key;
}

bool NodeData::operator>(const NodeData &right) const{
    return this->key > right.key;
}


Node::Node(NodeData data){
    this->data = data;
    this->left = NULL;
    this->right = NULL;
}

void Node::insert(NodeData data){
    if(data < this->data){
        if (!this->left){
            this->left = std::make_unique<Node>(data);
        }else{
            this->left->insert(data);
        }
    }else{
        if (!this->right){
            this->right = std::make_unique<Node>(data);
        }else{
            this->right->insert(data);
        }
    }
}

void Node::printTree(){
    if(this->left){
        this->left->printTree();
    }
    std::cout <<"[" <<this->data.key << ", " << this->data.data<<"]" << std::endl;
    if(this->right){
        this->right->printTree();
    }
}

int Node::sortedUntil(int k, std::vector<NodeData>& sorted){
    if(k==0){
        return k;
    }
    if(this->left){
        this->left->sortedUntil(k, sorted);
    }
    if(k>0){
        sorted.push_back(this->data);
        k--;
    }
    if(this->right){
        this->right->sortedUntil(k, sorted);
    }
    return k;
}
