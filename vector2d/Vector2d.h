#ifndef _Vector2d_hpp_
#define _Vector2d_hpp_

#include <iostream>

template<typename T>
class Vector2d{
public:
	//constructors
	Vector2d(T ix, T iy);
	Vector2d(T ia);
	Vector2d();

	//variables
	T x,y;

	//operators
	Vector2d operator+(const Vector2d &right) const;
	Vector2d operator-(const Vector2d &right) const;
	Vector2d operator*(const Vector2d &right) const;
	Vector2d operator/(const Vector2d &right) const;
	Vector2d& operator+=(const Vector2d &right);
	Vector2d& operator-=(const Vector2d &right);
	Vector2d& operator*=(const Vector2d &right);
	Vector2d& operator/=(const Vector2d &right);
	T operator[](int index) const;
	

	//functions
	T getX() const;
	T getY() const;

	/**
	 * @brief Change the vector length to exactly one
	 */
	void normalize();
	/**
	 * @brief Rotate the vector clockwize in the z direction
	 * 
	 * @param rotation Rotation angle in radians.
	 */
	void rotate(T rotation);
	/**
	 * @brief Rotate the vector clockwize in the z direction
	 * 
	 * @param rotation Rotation angle in radians.
	 */
	Vector2d rotated(T rotation) const;
	/**
	 * @brief Return a vector that is the normelized vector of this vector
	 * @details [long description]
	 * @return this vector normilized
	 */
	Vector2d normalized() const;
	/**
	 * @brief Calcultes the length of the vector and returns it.
	 * @details [long description]
	 * @return The lenght of this vector
	 */
	T absolute() const;
	/**
	 * @brief Calculate the dot product of two vetors.
	 * @details The dot product is the product of the vector in the
	 * same length.
	 * 
	 * @param left Left hand side of the dot operator.
	 * @param right Right hand side of the dot operator.
	 * 
	 * @return Vector result of the dot operation.
	 */
	static T dotProduct(const Vector2d &left, const Vector2d &right);
    static T crossProduct(const Vector2d &left, const Vector2d &right);

	static T euclideanDistance(const Vector2d &left, const Vector2d &right);
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2d<T>& vector2);
#endif //_Vector2d_hpp_