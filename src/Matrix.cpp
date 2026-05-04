#include"Matrix.h"
#include<Eigen/Dense>
//#include<iostream>
#include<cmath>

Eigen::Matrix2f Matrix::rotate(float phi)
{
	Eigen::Matrix2f temp;
	temp << cos(phi), -sin(phi),
			sin(phi), cos(phi);
	return temp;
}

Eigen::Matrix2f Matrix::rotateAbleit(float angle)
{
	Eigen::Matrix2f temp;
	temp << -sin(angle), -cos(angle),
			cos(angle), -sin(angle);
	return temp;
}



Eigen::Vector2f Vector::triproduct(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C)
{
	return A.dot(C) * B - A.dot(B) * C;
}



float math2D::getDistance(const Eigen::Vector2f& A, const Eigen::Vector2f& B)
{
	Eigen::Vector2f side = B - A;
	Eigen::Vector2f norm = { side[1], -side[0] };
	return std::abs(A.dot(norm.normalized()));
}

Eigen::Vector2f math2D::getNormal(const Eigen::Vector2f& A, const Eigen::Vector2f& B)
{
	Eigen::Vector2f side = B - A;
	Eigen::Vector2f norm;
	norm << side[1], -side[0];
	if (norm.dot(A) > 0)
	{
		norm = -norm;
	}
	return norm;
}

float math2D::getPLDistance(const Eigen::Vector2f& point, const Eigen::Vector2f& pos, const Eigen::Vector2f& perp)
{
	Eigen::Vector2f temp = point - pos;
	Eigen::Vector2f norm;
	norm << perp[1], -perp[0];
	return temp.dot(norm.normalized());
}

Eigen::Vector2f math2D::getIntersectPoint(
	const Eigen::Vector2f& posA, const Eigen::Vector2f& perpA, 
	const Eigen::Vector2f& posB, const Eigen::Vector2f& perpB)
{
	Eigen::Matrix2f M;
	M.col(0) = perpA;
	M.col(1) = -perpB;

	Eigen::Vector2f res = M.inverse() * (posB - posA);
	return posA + res[0] * perpA;
}

Eigen::Vector2f math2D::getProjection(const Eigen::Vector2f& point, const Eigen::Vector2f& pos, const Eigen::Vector2f& perp)
{
	return pos + (point - pos).dot(perp.normalized()) * perp;
}

float math2D::cross(const Eigen::Vector2f& vec1, const Eigen::Vector2f& vec2)
{
	return(vec1[0] * vec2[1] - vec1[1] * vec2[0]);
}

float math2D::cross(const Eigen::Vector2f& vec1, const Eigen::Vector2f& vec2, const Eigen::Vector2f& vec3)
{
	return math2D::cross(vec2-vec1, vec3-vec1);
}

bool math2D::isIntersct(
	const Eigen::Vector2f& A, const Eigen::Vector2f& B,
	const Eigen::Vector2f& C, const Eigen::Vector2f& D)
{
	//A,B在CD两侧
	bool a = cross(A - C, D - C) * cross(B - C, D - C) < 0;
	//C,D在AB两侧
	bool b = cross(C - A, B - A) * cross(D - A, B - A) < 0;
	return a && b;
}

bool math2D::inTriangle(
	const Eigen::Vector2f& P, const Eigen::Vector2f& A,
	const Eigen::Vector2f& B, const Eigen::Vector2f& C)
{
	float c1 = cross(A, B, P);
	float c2 = cross(B, C, P);
	float c3 = cross(C, A, P);
	return (c1 > 0 && c2 > 0 && c3 > 0) || (c1 < 0 && c2 < 0 && c3 < 0);
}