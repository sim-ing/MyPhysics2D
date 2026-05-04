#include"Collision.h"
#include"Matrix.h"
#include"test.h"
#include<iostream>
#include<cmath>
//#include<Eigen/Dense>




//SupportPoint
SupportPoint::SupportPoint() {}
SupportPoint::SupportPoint(const SupportPoint& other) :point(other.point), supportA(other.supportA), supportB(other.supportB) {}




//GJK
Eigen::Vector2f GJK::getNorm(const SupportPoint& A, const SupportPoint& B) const
{
	return math2D::getNormal(A.point, B.point);
}

bool GJK::reachOrigin(const SupportPoint& A, const Eigen::Vector2f& norm) const
{
	return (A.point.dot(norm) > 0);
}

int GJK::inArea(Eigen::Vector2f& norm) const
{

	int a = (lastPoint + 1) % 3;
	int b = (lastPoint + 2) % 3;

	//norm一定向simplex外侧
	Eigen::Vector2f norma = getNorm(simplex[lastPoint], simplex[a]);
	if (norma.dot(simplex[b].point - simplex[lastPoint].point) > 0)
	{
		norma = -norma;
	}
	//std::cout << "norm a:\n" <<norma<< std::endl;

	Eigen::Vector2f normb = getNorm(simplex[lastPoint], simplex[b]);
	if (normb.dot(simplex[a].point - simplex[lastPoint].point) > 0)
	{
		normb = -normb;
	}
	//std::cout << "norm b:\n" << normb << std::endl;

	if ((-simplex[a].point.dot(norma) > 0) &&
		(-simplex[lastPoint].point.dot(norma) > 0))
	{
		norm = norma;
		return a;
	}
	else if ((-simplex[b].point.dot(normb) > 0) &&
		(-simplex[lastPoint].point.dot(normb) > 0))
	{
		norm = normb;
		return b;
	}
	else
	{
		return lastPoint;
	}
	//AO与ABnorm夹角小于九十度
}

const std::array<SupportPoint, 3>& GJK::getSimplex() const
{
	return simplex;
}

GJK::State GJK::getState() const
{
	return state;
}

Eigen::Vector2f GJK::getNextNorm() const
{
	return nextNorm;
}

void GJK::step(const SupportPoint& newpoint)
{
	//std::cout << "nextNorm from last round:\n" << nextNorm << std::endl;
	//std::cout << "start gjk step:\nnewpoint:\n" << newpoint << std::endl;
	switch (simplexCount)
	{
	case 0:
		simplex[0] = newpoint;
		nextNorm = -newpoint.point;
		simplexCount = 1;
		//std::cout << simplex << "\n" << std::endl;
		break;
	case 1:
		if (!reachOrigin(newpoint, nextNorm))
		{
			state = State::Nointersect;
			return;
		}
		simplex[1] = newpoint;
		nextNorm = getNorm(simplex[0], simplex[1]);
		lastPoint = 2;
		simplexCount = 2;
		//std::cout << simplex << "\n" << std::endl;
		break;
	case 2:
		simplexCount = 3;
	case 3:
		if (!reachOrigin(newpoint, nextNorm))
		{
			state = State::Nointersect;
			return;
		}

		simplex[lastPoint] = newpoint;
		//std::cout << simplex << "\n" << std::endl;

		int temp = inArea(nextNorm);
		if (temp == lastPoint)
		{
			state = State::Intersect;
			return;
		}
		else
		{
			lastPoint = temp;
		}
		/*
		//原点在02方向朝外
		if (inArea(a))
		{
			nextNorm = getNorm(simplex[a], simplex[lastPoint]);
			lastPoint = b;
			std::cout << "\noutside a\n" << std::endl;
		}
		//原点在12方向朝外
		else if (inArea(b))
		{
			nextNorm = getNorm(simplex[b], simplex[lastPoint]);
			lastPoint = a;
			std::cout << "\noutside b\n" << std::endl;
		}
		//都不在，找到了
		else
		{
			state = State::Intersect;
			return;
		}
		*/
		break;
	}
}


//EPA
EPA::EPA(const std::array<SupportPoint, 3>& simplex)
{
	polytope.resize(3);
	for (int i = 0; i <= 2; i++)
	{
		polytope[i] = simplex[i];
	}
	//polySize = 3;
	lastPoint = findSide();
	nextNorm = getNorm(lastPoint);
}

Eigen::Vector2f EPA::getNextNorm() const
{
	return nextNorm;
}

bool EPA::getState() const
{
	return state;
}

float EPA::getDepth() const
{
	return conDepth;
}

void EPA::addPoint(int index, const SupportPoint& newPoint)
{
	polytope.insert(polytope.begin() + index + 1, newPoint);
}

void EPA::repPoint(int index, const SupportPoint& newPoint)
{
	polytope[index] = newPoint;
}

Eigen::Vector2f EPA::getNorm(int index)
{
	int a = index % polySize;
	int b = (index + 1) % polySize;
	//用于判断朝外方向，原点在相切时会爆炸
	int c = (index + 2) % polySize;

	Eigen::Vector2f perp = polytope[a].point - polytope[b].point;
	Eigen::Vector2f norm = { perp[1], -perp[0] };

	if (norm.dot(polytope[c].point - polytope[b].point) > 0)
	{
		norm = -norm;
	}
	return norm;
}

float EPA::getDistance(int index) const
{
	int a = index % polySize;
	int b = (index + 1) % polySize;
	return math2D::getDistance(polytope[a].point, polytope[b].point);
}

int EPA::findSide()
{
	int index = 0;
	float dis = getDistance(0), tempdis = 0;
	for (int i = 1; i < polySize; i++)
	{
		tempdis = getDistance(i);
		if (dis > tempdis)
		{
			dis = tempdis;
			index = i;
		}
	}
	conDepth = dis;
	return index;
}

void EPA::step(const SupportPoint& newpoint)
{
	//std::cout << "\n\n<<<<<<data this round: >>>>>>\n"<< std::endl;
	//std::cout << polytope << std::endl;
	//std::cout << "nextNorm :\n" << nextNorm << std::endl;
	//std::cout << "newpoint :\n" << newpoint << std::endl;
	//std::cout << "lastPoint :\n" << lastPoint <<"\n"<< std::endl;


	//判断新找到的点在polytope上面
	//Eigen::Vector2f temp = newpoint.point - polytope[lastPoint].point;
	float d = newpoint.point.dot(nextNorm.normalized());
	//std::cout << "end? : " << temp.dot(nextNorm) << std::endl;
	//std::cout << "conDepth:" << conDepth << std::endl;
	//std::cout <<"d:" << d << std::endl;
	if (abs(d - conDepth) <= eps)
	{
		state = true;
	}
	else
	{
		addPoint(lastPoint, newpoint);
		//在多边形中找到最短边的引索
		lastPoint = findSide();
		nextNorm = getNorm(lastPoint);
		polySize++;
	}
}
