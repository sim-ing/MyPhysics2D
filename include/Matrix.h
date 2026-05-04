#pragma once
#include<Eigen/Core>

namespace Matrix
{
	Eigen::Matrix2f rotate(float angle);

	Eigen::Matrix2f rotateAbleit(float angle);
}

namespace Vector
{
	//A x B x C
	Eigen::Vector2f triproduct(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C);
}

namespace math2D
{

	//给出边距离原点的距离
	float getDistance(const Eigen::Vector2f& A, const Eigen::Vector2f& B);

	//给出朝向原点的两点法向量（不归一处理）
	Eigen::Vector2f getNormal(const Eigen::Vector2f& A, const Eigen::Vector2f& B);

	//点到直线的距离,在方向向量左侧为负
	float getPLDistance(const Eigen::Vector2f& point, const Eigen::Vector2f& pos, const Eigen::Vector2f& perp);

	//求交点
	Eigen::Vector2f getIntersectPoint(
		const Eigen::Vector2f& posA, const Eigen::Vector2f& perpA, 
		const Eigen::Vector2f& posB, const Eigen::Vector2f& perpB);

	//找投影
	Eigen::Vector2f getProjection(const Eigen::Vector2f& point, const Eigen::Vector2f& pos, const Eigen::Vector2f& perp);

	//二维叉乘：vec1 x vec2
	float cross(const Eigen::Vector2f& vec1, const Eigen::Vector2f& vec2);

	//直接从三点找叉乘,vec1为共有起点
	float cross(const Eigen::Vector2f& vec1, const Eigen::Vector2f& vec2, const Eigen::Vector2f& vec3);

	//判断AB与CD线段是否相交
	bool isIntersct(
		const Eigen::Vector2f& A, const Eigen::Vector2f& B,
		const Eigen::Vector2f& C, const Eigen::Vector2f& D);

	//判断点P是否在三角形ABC内
	bool inTriangle(
		const Eigen::Vector2f& P, const Eigen::Vector2f& A,
		const Eigen::Vector2f& B, const Eigen::Vector2f& C);
}