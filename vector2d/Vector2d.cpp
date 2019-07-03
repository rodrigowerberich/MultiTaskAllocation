#include "Vector2d.h"
#include <cmath>

template<typename T>
Vector2d<T>::Vector2d(T ix, T iy): x(ix), y(iy){}
template<typename T>
Vector2d<T>::Vector2d(T ia): x(ia), y(ia){}
template<typename T>
Vector2d<T>::Vector2d(): x(0), y(0){}

template<typename T>
Vector2d<T> Vector2d<T>::operator+(const Vector2d<T> &right) const{
	Vector2d<T> temp(*this);
	temp += right;
	return temp;
}
template<typename T>
Vector2d<T> Vector2d<T>::operator-(const Vector2d<T> &right) const{
	Vector2d<T> temp(*this);
	temp -= right;
	return temp;
}
template<typename T>
Vector2d<T> Vector2d<T>::operator*(const Vector2d<T> &right) const{
	Vector2d<T> temp(*this);
	temp *= right;
	return temp;
}
template<typename T>
Vector2d<T> Vector2d<T>::operator/(const Vector2d<T> &right) const{
	Vector2d<T> temp(*this);
	temp /= right;
	return temp;
}
template<typename T>
Vector2d<T>& Vector2d<T>::operator+=(const Vector2d<T> &right){
	x += right.x;
	y += right.y;
	return *this;
}
template<typename T>
Vector2d<T>& Vector2d<T>::operator-=(const Vector2d<T> &right){
	x -= right.x;
	y -= right.y;
	return *this;
}
template<typename T>
Vector2d<T>& Vector2d<T>::operator*=(const Vector2d<T> &right){
	x *= right.x;
	y *= right.y;
	return *this;
}
template<typename T>
Vector2d<T>& Vector2d<T>::operator/=(const Vector2d<T> &right){
	if(right.x != 0 && right.y != 0){
		x /= right.x;
		y /= right.y;
	}
	return *this;
}
template<typename T>
void Vector2d<T>::normalize(){
	if(x != 0 || y != 0){
		T length = sqrt(pow(x, 2) + pow(y, 2));
		x /= length;
		y /= length;
	}
}
template<typename T>
T Vector2d<T>::getX() const{
	return x;
}

template<typename T>
T Vector2d<T>::getY() const{
	return y;
}


template<typename T>
void Vector2d<T>::rotate(T rotation){
	T x_1 = x;
	x = (x * cos(rotation)) - (y * sin(rotation));
	y = (x_1 * sin(rotation)) + (y * cos(rotation));
}
template<typename T>
Vector2d<T> Vector2d<T>::rotated(T rotation) const{
	return Vector2d(
		(x * cos(rotation)) - (y * sin(rotation)),
		(x * sin(rotation)) + (y * cos(rotation))
	);
}
template<typename T>
Vector2d<T> Vector2d<T>::normalized() const{
	if(x != 0 || y != 0){
		T length = sqrt(pow(x, 2) + pow(y, 2));
		return Vector2d<T>(
			x / length,
			y / length);
	}
	else{
		return Vector2d<T>(0, 0);
	}
}
template<typename T>
T Vector2d<T>::absolute() const{
	return sqrt(pow(x, 2) + pow(y, 2));
}
template<typename T>
T Vector2d<T>::dotProduct(const Vector2d<T> &left, const Vector2d<T> &right){
	return left.x * right.x + left.y * right.y;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2d<T>& vector2){
    os << '{' << vector2.x << ',' << vector2.y << '}';
    return os;
}

//compile the following classes and function using the templates

//Compile for double
template class Vector2d<double>;
template std::ostream& operator<<(std::ostream& os, const Vector2d<double>& vector2);
//Compile for float
template class Vector2d<float>;
template std::ostream& operator<<(std::ostream& os, const Vector2d<float>& vector2);