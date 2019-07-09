#ifndef MAPPED_VECTOR_OF_POINTERS__
#define MAPPED_VECTOR_OF_POINTERS__

#include <vector>
#include <map>
#include <memory>

template <class base_type>
class MappedVectorOfPointers {
private:
    std::vector<std::unique_ptr<base_type>> m_values;
    std::map<std::string, size_t> m_name_map;
    static std::unique_ptr<base_type> m_null;
public:
    const std::unique_ptr<base_type> & getValueByName(const std::string& value_name){
        if(m_name_map.count(value_name) > 0){
            return m_values[m_name_map[value_name]];
        }else{
            return m_null;
        }
    }
    // begin
    typename std::vector<std::unique_ptr<base_type>>::iterator begin() noexcept{
        return m_values.begin();
    }
    typename std::vector<std::unique_ptr<base_type>>::const_iterator begin() const noexcept{
        return m_values.begin();
    }
    // end
    typename std::vector<std::unique_ptr<base_type>>::iterator end() noexcept {
        return m_values.end();
    }
    typename std::vector<std::unique_ptr<base_type>>::const_iterator end() const noexcept {
        return m_values.end();
    }
    // rbegin
    typename std::vector<std::unique_ptr<base_type>>::reverse_iterator rbegin() noexcept {
        return m_values.rbegin();
    }
    typename std::vector<std::unique_ptr<base_type>>::const_reverse_iterator rbegin() const noexcept {
        return m_values.rbegin();
    }
    // rend
    typename std::vector<std::unique_ptr<base_type>>::reverse_iterator rend() noexcept {
        return m_values.rend();
    }
    typename std::vector<std::unique_ptr<base_type>>::const_reverse_iterator rend() const noexcept {
        return m_values.rend();
    }

    // size
    size_t size() const noexcept {
        return m_values.size();
    }
    // empty
    bool empty() const noexcept {
        return m_values.empty();
    }

    // // operator []
    // std::unique_ptr<base_type>& operator[] (size_t n){
    //     return m_values[n];
    // }
    // const std::unique_ptr<base_type>& operator[] (size_t n) const{
    //     return m_values[n];
    // }
    // // at
    // std::unique_ptr<base_type>& at (size_t n){
    //     return m_values.at(n);
    // }
    // const std::unique_ptr<base_type>& at (size_t n) const{
    //     return m_values.at(n);
    // }
    // // front
    // std::unique_ptr<base_type>& front() {
    //     return m_values.front();
    // }
    // const std::unique_ptr<base_type>& front() const {
    //     return m_values.front();
    // }
    // // back
    // std::unique_ptr<base_type>& back() {
    //     return m_values.back();
    // }
    // const std::unique_ptr<base_type>& back() const {
    //     return m_values.back();
    // }
    // // data
    // std::unique_ptr<base_type>* data() noexcept {
    //     return m_values.data();
    // }
    // const std::unique_ptr<base_type>* data() const noexcept {
    //     return m_values.data();
    // }

    // // assign 
    // template <class InputIterator>
    // void assign (InputIterator first, InputIterator last){
    //     m_values.assign(first, second);
    // }
    // void assign (size_t n, const std::unique_ptr<base_type>& val){
    //     m_values.assign(n, val);
    // }
    // push_back
    // void push_back(const std::unique_ptr<base_type>& val) {
    //     m_values.push_back(std::move(val));
    // }
    void push_back(std::unique_ptr<base_type>&& val) {
        int position = size();
        auto name = val->getName();
        m_values.push_back(std::move(val));
        m_name_map.emplace(name, position);
    }
    // // pop_back
    // void pop_back() {
    //     m_values.pop_back();
    // }
    // // erase
    // std::vector<std::unique_ptr<base_type>>::iterator erase (std::vector<std::unique_ptr<base_type>>::const_iterator position){
    //     return m_values.erase(position);
    // }
    // std::vector<std::unique_ptr<base_type>>::iterator erase (std::vector<std::unique_ptr<base_type>>::const_iterator first, std::vector<std::unique_ptr<base_type>>::const_iterator last){
    //     return m_values.erase(first, last);
    // }
    // // swap
    // void swap (std::vector<std::unique_ptr<base_type>>& x){
    //     m_values.swap(x);
    // }
    // // clear
    // void clear() noexcept {
    //     m_values.clear();
    // }
    // // emplace
    // template <class... Args>
    // std::vector<std::unique_ptr<base_type>>::iterator emplace (std::vector<std::unique_ptr<base_type>>::const_iterator position, Args&&... args){
    //     m_values.emplace(position, args);
    // }
    // // emplace_back
    // template <class... Args>
    // void emplace_back (Args&&... args){
    //     m_values.emplace_back(args);
    // }
};

template <class base_type> std::unique_ptr<base_type> MappedVectorOfPointers<base_type>::m_null = nullptr;


#endif // MAPPED_VECTOR_OF_POINTERS__