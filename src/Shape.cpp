#include "Shape.h"
//#include<iostream>
#include<algorithm>
#include<Eigen/Dense>
#include<cmath>
#include<memory>
#include"Matrix.h"
#include"Collider.h"


Edge::Edge(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& norm, int index) : id(index)
{
	normal = norm.normalized();
	if (math2D::getPLDistance(p1, p2, norm) < 0)
	{
		pointA = p1;
		pointB = p2;
	}
	else
	{
		pointB = p1;
		pointA = p2;
	}
	perp = (pointB - pointA).normalized();
}

void Edge::transform(const Eigen::Matrix2f& R, const Eigen::Vector2f& T)
{
	pointA = R * pointA + T;
	pointB = R * pointB + T;
	normal = (R * normal).normalized();
	perp = (pointB - pointA).normalized();
}

bool AABB_overlap(const AABB& rA, const AABB& rB)
{
	if ((rA.x_min > rB.x_max) || (rA.x_max < rB.x_min)) {
		return false;
	}
	else if ((rA.y_min > rB.y_max) || (rA.y_max < rB.y_min)) {
		return false;
	}
	else{
		return true;
	}
}

Vec2View::Vec2View(const Eigen::Vector2f* ptr, int c):vertice(ptr), size(c){}

int Vec2View::count()const
{
	return size;
}

std::vector<Eigen::Vector2f> monotonChain(std::vector<Eigen::Vector2f> vertices)
{
	int size = vertices.size();
	if (size <= 2) { return vertices; }

	std::sort(vertices.begin(), vertices.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b)
		{ return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]); });

	std::vector<Eigen::Vector2f> convex;

	for (int i = 0; i < size; i++)
	{
		while (convex.size() >= 2 &&
			math2D::cross(convex[convex.size() - 2], convex.back(), vertices[i]) <= 0)
		{
			convex.pop_back();
		}
		convex.push_back(vertices[i]);
	}

	int l_size = convex.size();
	for (int i = size - 2; i >= 0; i--)
	{
		while (convex.size() > l_size &&
			math2D::cross(convex[convex.size() - 2], convex.back(), vertices[i]) <= 0)
		{
			convex.pop_back();
		}
		convex.push_back(vertices[i]);
	}

	convex.pop_back();
	return convex;
}

float calArea(const std::vector<Eigen::Vector2f>& ver)
{
	int size = ver.size();
	float Area = 0;
	for (int i = 0; i < size; i++)
	{
		Area += math2D::cross(ver[i],ver[(i + 1) % size]);
	}
	Area /= 2;
	return Area;
}

Eigen::Matrix2f calInertia(const std::vector<Eigen::Vector2f>& ver)
{
	int size = ver.size();
	Eigen::Matrix2f I = Eigen::Matrix2f::Zero();
	for (int i = 0; i < size; i++)
	{
		int j = (i + 1) % size;
		float a = math2D::cross(ver[i], ver[j]);
		I(1, 1) += a * (ver[i][0] * ver[i][0] + ver[i][0] * ver[j][0] + ver[j][0] * ver[j][0]);
		I(0, 0) += a * (ver[i][1] * ver[i][1] + ver[i][1] * ver[j][1] + ver[j][1] * ver[j][1]);
		I(1, 0) += a * (ver[i][0] * ver[j][1] + 2 * ver[i][0] * ver[i][1] + 2 * ver[j][0] * ver[j][1] + ver[j][0] * ver[i][1]);
	}
	I(0, 0) /= 12;
	I(1, 1) /= 12;
	I(1, 0) /= 24;
	I(0, 0) = abs(I(0, 0));
	I(1, 1) = abs(I(1, 1));
	I(1, 0) = abs(I(1, 0));
	I(0, 1) = I(1, 0);
	return I;
}

Eigen::Vector2f calCenter(const std::vector<Eigen::Vector2f>& ver, float Area)
{
	int n = ver.size();

	float cx = 0.0f, cy = 0.0f;
	for (int i = 0; i < n; i++)
	{
		float a = math2D::cross(ver[i], ver[(i + 1) % n]);
		cx += a * (ver[i][0] + ver[(i + 1) % n][0]);
		cy += a * (ver[i][1] + ver[(i + 1) % n][1]);
	}

	cx = cx / (6 * Area);
	cy = cy / (6 * Area);

	return { cx,cy };
}








EarCut::EarCut(const std::vector<Eigen::Vector2f>& ver):vertices(ver)
{
	int n = ver.size();
	nodes.resize(n);

	for (int i = 0; i < n; i++)
	{
		Vertice t;
		t.prev = (i - 1 + n) % n;
		t.index = i;
		t.next = (i + 1) % n;
		t.isConvex = math2D::cross(ver[t.prev], ver[t.index], ver[t.next]) > 0;
		if (!t.isConvex)
		{
			concaveNodes.push_back(i);
		}

		nodes[i] = t;
	}
}

void EarCut::deleteVertice(int index)
{
	int n = nodes[index].next;
	int p = nodes[index].prev;
	nodes[n].prev = p;
	nodes[p].next = n;
	triangles.push_back(nodes[index]);

	//检查是凹点是否被消除
	if ((!nodes[n].isConvex) && 
		(math2D::cross(vertices[p], vertices[n], vertices[nodes[n].next]) > 0))
	{
		nodes[n].isConvex = true;
		concaveNodes.erase(
			std::remove(concaveNodes.begin(), concaveNodes.end(), n), 
			concaveNodes.end());
	}
	if ((!nodes[p].isConvex) &&
		(math2D::cross(vertices[nodes[p].prev], vertices[p], vertices[n]) > 0))
	{
		nodes[p].isConvex = true;
		concaveNodes.erase(
			std::remove(concaveNodes.begin(), concaveNodes.end(), p),
			concaveNodes.end());
	}
}

bool EarCut::isEar(int index)
{
	int p = nodes[index].prev;
	int n = nodes[index].next;
	if (!nodes[index].isConvex)
	{
		return false;
	}

	for (int i : concaveNodes)
	{
		if (i == p || i == n) continue;
		if (math2D::inTriangle(vertices[i], vertices[p], vertices[index], vertices[n]))
		{
			return false;
		}
	}
	return true;
}

void EarCut::cut()
{
	int timeout = nodes.size() * nodes.size();
	int check = 0;
	while (concaveNodes.size() != 0 && timeout-- > 0)
	{
		if (isEar(check))
		{
			deleteVertice(check);
		}
		check = nodes[check].next;
	}
	
	//只剩下凸多边形了
	while (nodes[check].next != nodes[check].prev)
	{
		deleteVertice(check);
		check = nodes[check].next;
	}
}

const std::vector<Vertice>& EarCut::getTriangles() const
{
	return triangles;
}



//默认构造函数，三点必须是以形心为原点的
Triangle::Triangle(const std::array<Eigen::Vector2f, 3>& ver):
	vertices(ver)
{
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		normals[i] = -math2D::getNormal(vertices[i], vertices[j]).normalized();
	}
}

//通过三个点来确定三角形的静态函数，同时反馈形心位置给rigidbody
Triangle Triangle::createFromPoints(const std::array<Eigen::Vector2f, 3>& ver, Eigen::Vector2f* cen)
{
	Eigen::Vector2f center;
	center = (ver[0] + ver[1] + ver[2]) / 3;
	if (cen != nullptr)
	{
		*cen = center;
	}
	return Triangle({ ver[0] - center, ver[1] - center, ver[2] - center });
}

//未来计划更多的特殊三角形构造方法

float Triangle::getArea() const
{
	Eigen::Vector2f lineAB, lineAC;
	lineAB = vertices[1] - vertices[0];
	lineAC = vertices[2] - vertices[0];
	return abs(lineAB.cross(lineAC) / 2);
}

Eigen::Matrix2f Triangle::getInertia() const
{
	Eigen::Matrix2f I = Eigen::Matrix2f::Zero();
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		float a = vertices[i][0] * vertices[j][1] - vertices[j][0] * vertices[i][1];
		I(1, 1) += a * (vertices[i][0] * vertices[i][0] + vertices[i][0] * vertices[j][0] + vertices[j][0] * vertices[j][0]);
		I(0, 0) += a * (vertices[i][1] * vertices[i][1] + vertices[i][1] * vertices[j][1] + vertices[j][1] * vertices[j][1]);
		I(1, 0) += a * (vertices[i][0] * vertices[j][1] + 2 * vertices[i][0] * vertices[i][1] + 2 * vertices[j][0] * vertices[j][1] + vertices[j][0] * vertices[i][1]);
	}
	I(0, 0) /= 12;
	I(1, 1) /= 12;
	I(1, 0) /= 24;
	I(0, 0) = abs(I(0, 0));
	I(1, 1) = abs(I(1, 1));
	I(1, 0) = abs(I(1, 0));
	I(0, 1) = I(1, 0);
	return I;
}

void Triangle::createProxy(Collider* col, 
	const Eigen::Vector2f& pos, float rot) const
{
	col->addProxy(std::make_unique<PolygonProxy>(col, this, pos, rot));
}

Eigen::Vector2f Triangle::getSupport(const Eigen::Vector2f& dir) const
{
	std::array<float, 3> distance;
	for (int i = 0; i <= 2; i++)
	{
		distance[i] = dir.dot(vertices[i]);
	}
	auto max = std::max_element(distance.begin(), distance.end());
	int index = std::distance(distance.begin(), max);
	return vertices[index];
}

Edge Triangle::getBestEdge(const Eigen::Vector2f& norm) const
{
	float product = norm.dot(normals[0]), temp = 0;
	int index = 0;

	for (int i = 1; i < 3; i++)
	{
		temp = norm.dot(normals[i]);
		if (temp > product)
		{
			index = i;
			product = temp;
		}
	}

	return Edge(vertices[index], vertices[(index + 1) % 3], normals[index], index);
}

AABB Triangle::getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const
{
	Eigen::Vector2f temp = rot * vertices[0] + pos;
	float xMin = temp[0];//x轴坐标
	float xMax = temp[0];
	float yMin = temp[1];//y轴坐标
	float yMax = temp[1];

	for (int i = 1; i < 3; i++)
	{
		temp = rot * vertices[i] + pos;
		xMin = std::min(temp[0], xMin);
		xMax = std::max(temp[0], xMax);
		yMin = std::min(temp[1], yMin);
		yMax = std::max(temp[1], yMax);
	}

	return AABB{ xMin, xMax, yMin, yMax };
}

Vec2View Triangle::getLVertices() const
{
	Vec2View view(vertices.data(), 3);
	return view;
}

const Eigen::Vector2f& Triangle::getLVertice(int index) const
{
	return vertices[index];
}

Vec2View Triangle::getLNormals() const
{
	Vec2View view(normals.data(), 3);
	return view;
}

const Eigen::Vector2f& Triangle::getLNormal(int index) const
{
	return normals[index];
}



//只有可信来源才能用的内部构造函数
Polygon::Polygon(const std::vector<Eigen::Vector2f>& ver, TrustSource):vertices(ver), size(ver.size())
{
	normals.resize(size);
	for (int i = 0; i < size; i++)
	{
		int j = (i + 1) % size;
		normals[i] = -math2D::getNormal(vertices[i], vertices[j]).normalized();
	}
}

//默认构造函数，点必须是以形心为原点的
Polygon::Polygon(const std::vector<Eigen::Vector2f>& ver)
{
	vertices = monotonChain(ver);
	if (vertices.size() < ver.size())
	{
		throw"concave polygon";
	}
	size = vertices.size();
	normals.resize(size);
	for (int i = 0; i < size; i++)
	{
		int j = (i + 1) % size;
		normals[i] = -math2D::getNormal(vertices[i], vertices[j]).normalized();
	}
}

//通过三个点来确定三角形的静态函数，同时反馈形心位置给rigidbody
Polygon Polygon::createFromPoints(const std::vector<Eigen::Vector2f>& ver, Eigen::Vector2f* cen)
{
	auto t_ver = monotonChain(ver);
	int t_size = t_ver.size();
	if (t_size < ver.size())
	{
		throw"concave polygon";
	}

	//找到形心
	Eigen::Vector2f center = calCenter(ver, calArea(ver));
	if (cen != nullptr)
	{
		*cen = center;
	}

	//转换为形心为原点的坐标系
	for (int i = 0; i < t_size; i++)
	{
		t_ver[i] = t_ver[i] - center;
	}
	return Polygon(t_ver, TrustSource());
}

float Polygon::getArea() const
{
	float Area = 0;
	for (int i = 0; i < size; i++)
	{
		Area += math2D::cross(vertices[i], vertices[(i + 1) % size]);
	}
	Area /= 2;
	return Area;
}

Eigen::Matrix2f Polygon::getInertia() const
{
	Eigen::Matrix2f I = Eigen::Matrix2f::Zero();
	for (int i = 0; i < size; i++)
	{
		int j = (i + 1) % size;
		float a = math2D::cross(vertices[i], vertices[j]);
		I(1, 1) += a * (vertices[i][0] * vertices[i][0] + vertices[i][0] * vertices[j][0] + vertices[j][0] * vertices[j][0]);
		I(0, 0) += a * (vertices[i][1] * vertices[i][1] + vertices[i][1] * vertices[j][1] + vertices[j][1] * vertices[j][1]);
		I(1, 0) += a * (vertices[i][0] * vertices[j][1] + 2 * vertices[i][0] * vertices[i][1] + 2 * vertices[j][0] * vertices[j][1] + vertices[j][0] * vertices[i][1]);
	}
	I(0, 0) /= 12;
	I(1, 1) /= 12;
	I(1, 0) /= 24;
	I(0, 0) = abs(I(0, 0));
	I(1, 1) = abs(I(1, 1));
	I(1, 0) = abs(I(1, 0));
	I(0, 1) = I(1, 0);
	return I;
}

void Polygon::createProxy(Collider* col,
	const Eigen::Vector2f& pos, float rot) const
{
	col->addProxy(std::make_unique<PolygonProxy>(col, this, pos, rot));
}

Eigen::Vector2f Polygon::getSupport(const Eigen::Vector2f& dir) const
{
	std::vector<float> distance;
	distance.resize(size);
	for (int i = 0; i < size; i++)
	{
		distance[i] = dir.dot(vertices[i]);
	}
	auto max = std::max_element(distance.begin(), distance.end());
	int index = std::distance(distance.begin(), max);
	return vertices[index];
}

Edge Polygon::getBestEdge(const Eigen::Vector2f& norm) const
{
	float product = norm.dot(normals[0]), temp = 0;
	int index = 0;

	for (int i = 1; i < size; i++)
	{
		temp = norm.dot(normals[i]);
		if (temp > product)
		{
			index = i;
			product = temp;
		}
	}

	return Edge(vertices[index], vertices[(index + 1) % size], normals[index], index);
}

AABB Polygon::getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const
{
	Eigen::Vector2f temp = rot * vertices[0] + pos;
	float xMin = temp[0];//x轴坐标
	float xMax = temp[0];
	float yMin = temp[1];//y轴坐标
	float yMax = temp[1];

	for (int i = 1; i < size; i++)
	{
		temp = rot * vertices[i] + pos;
		xMin = std::min(temp[0], xMin);
		xMax = std::max(temp[0], xMax);
		yMin = std::min(temp[1], yMin);
		yMax = std::max(temp[1], yMax);
	}

	return AABB{ xMin, xMax, yMin, yMax };
}

Vec2View Polygon::getLVertices() const
{
	Vec2View view(vertices.data(),size);
	return view;
}

const Eigen::Vector2f& Polygon::getLVertice(int index) const
{
	return vertices[index];
}

Vec2View Polygon::getLNormals() const
{
	Vec2View view(normals.data(), size);
	return view;
}

const Eigen::Vector2f& Polygon::getLNormal(int index) const
{
	return normals[index];
}



Circle::Circle(const float& rad): radius(rad)
{

}

//通过拖拽来实现圆形构造，同时反馈形心位置给rigidbody
Circle Circle::createFromDrag(const std::array<Eigen::Vector2f, 2>& drag, Eigen::Vector2f& cen)
{
	return Circle(1);
}


float Circle::getArea() const
{
	return 0;
}
Eigen::Matrix2f Circle::getInertia() const
{
	return Eigen::Matrix2f::Zero();
}
void Circle::createProxy(Collider* col, 
	const Eigen::Vector2f& pos, float rot) const
{
	col->addProxy(std::make_unique<PolygonProxy>(col, this, pos, rot));
}
Eigen::Vector2f Circle::getSupport(const Eigen::Vector2f& dir) const
{
	return Eigen::Vector2f(0, 0);
}
Edge Circle::getBestEdge(const Eigen::Vector2f& norm) const
{
	throw("error");
}
AABB Circle::getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const
{
	return AABB{ 0, 0, 0, 0 };
}
Vec2View Circle::getLVertices() const
{
	throw "errow";
}
const Eigen::Vector2f& Circle::getLVertice(int index) const
{
	throw "errow";
}
Vec2View Circle::getLNormals() const
{
	throw "error";
}
const Eigen::Vector2f& Circle::getLNormal(int index) const
{
	return Eigen::Vector2f(0, 0);
}


Compound::Child::Child(const std::shared_ptr<Shape>& sptr):shape(sptr){}

Compound::Compound(const std::vector<Eigen::Vector2f>& ver, const std::vector<Child>& chl, TrustSource):
	vertices(ver), children(chl), ver_size(ver.size()), chl_size(chl.size()){ }

//默认构造函数,只对多边形有效
Compound::Compound(const std::vector<Eigen::Vector2f>& ver)
{
	ver_size = ver.size();
	if (ver_size < 4) { throw "toooooo small"; }

	//判断是否自交
	for (int i = 1; i < ver_size - 2; i++)
	{
		for (int j = i + 2; j < ver_size; j++)
		{
			if (i == 0 && j == ver_size - 1) continue;
			if (math2D::isIntersct(ver[i], ver[(i + 1) % ver_size], ver[j], ver[(j + 1) % ver_size]))
			{
				throw"Compound self Intersect";
			}
		}
	}
	
	//把顺序一定转成逆时针
	float Area = calArea(ver);
	vertices = ver;
	if (Area < 0)
	{
		std::reverse(vertices.begin(), vertices.end());
		Area = -Area;
	}

	Eigen::Vector2f center = calCenter(vertices, Area);
	for (int i = 0; i < ver_size; i++)
	{
		vertices[i] = vertices[i] - center;
	}
	
	//做凸多边形拆分
	EarCut earCut(vertices);
	earCut.cut();
	const std::vector<Vertice>& triangles = earCut.getTriangles();

	chl_size = triangles.size();
	children.reserve(chl_size);
	for (const auto& it : triangles)
	{
		Eigen::Vector2f pos;
		children.push_back(Child(std::make_shared<Triangle>(
			Triangle::createFromPoints({
				vertices[it.prev],
				vertices[it.index],
				vertices[it.next] },
				&pos))));
		children.back().position = pos;
	}
}



//通过三个点来确定三角形的静态函数，同时反馈形心位置给rigidbody
Compound Compound::createFromPoints(const std::vector<Eigen::Vector2f>& ver, Eigen::Vector2f* cen)
{
	int n = ver.size();
	if (n < 4) { throw "toooooo small"; }

	//判断是否自交
	for (int i = 1; i < n - 2; i++)
	{
		for (int j = i + 2; j < n; j++)
		{
			if (i == 0 && j == n - 1) continue;
			if (math2D::isIntersct(ver[i], ver[(i + 1) % n], ver[j], ver[(j + 1) % n]))
			{
				throw"Compound self Intersect";
			}
		}
	}

	//把顺序一定转成逆时针
	float Area = calArea(ver);
	std::vector<Eigen::Vector2f> vertices = ver;
	if (Area < 0)
	{
		std::reverse(vertices.begin(), vertices.end());
		Area = -Area;
	}

	//形心为原点
	Eigen::Vector2f center = calCenter(vertices, Area);
	for (int i = 0; i < n; i++)
	{
		vertices[i] = vertices[i] - center;
	}
	if (cen != nullptr)
	{
		*cen = center;
	}

	//做凸多边形拆分
	EarCut earCut(vertices);
	earCut.cut();
	const std::vector<Vertice>& triangles = earCut.getTriangles();

	std::vector<Child> children;
	children.reserve(triangles.size());
	for (const auto& it : triangles)
	{
		Eigen::Vector2f pos;
		children.push_back(Child(std::make_shared<Triangle>(
			Triangle::createFromPoints({
				vertices[it.prev],
				vertices[it.index],
				vertices[it.next] },
				&pos))));
		children.back().position = pos;
	}

	return Compound(vertices, children, Compound::TrustSource());
}

float Compound::getArea() const
{
	return calArea(vertices);
}

Eigen::Matrix2f Compound::getInertia() const
{
	return calInertia(vertices);
}

void Compound::createProxy(Collider* col, 
	const Eigen::Vector2f& pos, float rot) const
{
	for (int i = 0; i < chl_size; i++)
	{
		children[i].shape->createProxy(col, children[i].position, children[i].rotation);
	}
}
Eigen::Vector2f Compound::getSupport(const Eigen::Vector2f& dir) const
{
	throw "not supposed to use this function";
}
Edge Compound::getBestEdge(const Eigen::Vector2f& norm) const
{
	throw "not supposed to use this function";
}
AABB Compound::getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const
{
	throw "not supposed to use this function";
}
Vec2View Compound::getLVertices() const
{
	Vec2View view(vertices.data(), ver_size);
	return view;
}

const Eigen::Vector2f& Compound::getLVertice(int index) const
{
	return vertices[index];
}
Vec2View Compound::getLNormals() const
{
	throw "not supposed to use this function";
}
const Eigen::Vector2f& Compound::getLNormal(int index) const
{
	throw "not supposed to use this function";
}