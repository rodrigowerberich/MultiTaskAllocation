#ifndef ROBOTS_H__
#define ROBOTS_H__

#include <vector>
#include <Robot.h>
#include <memory>
#include <algorithm>
#include <map>

// using Robots = std::vector<std::unique_ptr<Robot>>;

class Robots{
private:
    std::vector<std::unique_ptr<Robot>> m_robots;
    std::map<std::string, size_t> m_name_map;
public:
    const std::unique_ptr<Robot> & getRobot(const std::string& robot_name);

    // begin
    std::vector<std::unique_ptr<Robot>>::iterator begin() noexcept{
        return m_robots.begin();
    }
    std::vector<std::unique_ptr<Robot>>::const_iterator begin() const noexcept{
        return m_robots.begin();
    }
    // end
    std::vector<std::unique_ptr<Robot>>::iterator end() noexcept {
        return m_robots.end();
    }
    std::vector<std::unique_ptr<Robot>>::const_iterator end() const noexcept {
        return m_robots.end();
    }
    // rbegin
    std::vector<std::unique_ptr<Robot>>::reverse_iterator rbegin() noexcept {
        return m_robots.rbegin();
    }
    std::vector<std::unique_ptr<Robot>>::const_reverse_iterator rbegin() const noexcept {
        return m_robots.rbegin();
    }
    // rend
    std::vector<std::unique_ptr<Robot>>::reverse_iterator rend() noexcept {
        return m_robots.rend();
    }
    std::vector<std::unique_ptr<Robot>>::const_reverse_iterator rend() const noexcept {
        return m_robots.rend();
    }

    // size
    size_t size() const noexcept {
        return m_robots.size();
    }
    // empty
    bool empty() const noexcept {
        return m_robots.empty();
    }

    // // operator []
    // std::unique_ptr<Robot>& operator[] (size_t n){
    //     return m_robots[n];
    // }
    // const std::unique_ptr<Robot>& operator[] (size_t n) const{
    //     return m_robots[n];
    // }
    // // at
    // std::unique_ptr<Robot>& at (size_t n){
    //     return m_robots.at(n);
    // }
    // const std::unique_ptr<Robot>& at (size_t n) const{
    //     return m_robots.at(n);
    // }
    // // front
    // std::unique_ptr<Robot>& front() {
    //     return m_robots.front();
    // }
    // const std::unique_ptr<Robot>& front() const {
    //     return m_robots.front();
    // }
    // // back
    // std::unique_ptr<Robot>& back() {
    //     return m_robots.back();
    // }
    // const std::unique_ptr<Robot>& back() const {
    //     return m_robots.back();
    // }
    // // data
    // std::unique_ptr<Robot>* data() noexcept {
    //     return m_robots.data();
    // }
    // const std::unique_ptr<Robot>* data() const noexcept {
    //     return m_robots.data();
    // }

    // // assign 
    // template <class InputIterator>
    // void assign (InputIterator first, InputIterator last){
    //     m_robots.assign(first, second);
    // }
    // void assign (size_t n, const std::unique_ptr<Robot>& val){
    //     m_robots.assign(n, val);
    // }
    // push_back
    // void push_back(const std::unique_ptr<Robot>& val) {
    //     m_robots.push_back(std::move(val));
    // }
    void push_back(std::unique_ptr<Robot>&& val) {
        int position = size();
        auto name = val->getName();
        m_robots.push_back(std::move(val));
        m_name_map.emplace(name, position);
    }
    // // pop_back
    // void pop_back() {
    //     m_robots.pop_back();
    // }
    // // erase
    // std::vector<std::unique_ptr<Robot>>::iterator erase (std::vector<std::unique_ptr<Robot>>::const_iterator position){
    //     return m_robots.erase(position);
    // }
    // std::vector<std::unique_ptr<Robot>>::iterator erase (std::vector<std::unique_ptr<Robot>>::const_iterator first, std::vector<std::unique_ptr<Robot>>::const_iterator last){
    //     return m_robots.erase(first, last);
    // }
    // // swap
    // void swap (std::vector<std::unique_ptr<Robot>>& x){
    //     m_robots.swap(x);
    // }
    // // clear
    // void clear() noexcept {
    //     m_robots.clear();
    // }
    // // emplace
    // template <class... Args>
    // std::vector<std::unique_ptr<Robot>>::iterator emplace (std::vector<std::unique_ptr<Robot>>::const_iterator position, Args&&... args){
    //     m_robots.emplace(position, args);
    // }
    // // emplace_back
    // template <class... Args>
    // void emplace_back (Args&&... args){
    //     m_robots.emplace_back(args);
    // }
};


#endif // ROBOTS_H__