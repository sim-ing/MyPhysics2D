#include"Collider.h"
#include<Eigen/Dense>
#include"Matrix.h"
#include<algorithm>
//#include<iostream>


//先默认rb只有一个collider，以后再加多collider
Collider::Collider(RigidBody* b) :
	shape(b->getShape()), body(b), transform(b->getTrans())
{
	shape->createProxy(this);
	p_size = proxies.size();

	count = shape->getLVertices().count();

	//b->addCollider(this);

	vertices.resize(count);
	normals.resize(count);
	update();
}

const Eigen::Vector2f& Collider::getPos() const
{
	return transform->position;
}
float Collider::getRot() const
{
	return transform->rotation;
}
const Eigen::Matrix2f& Collider::getRotMat() const
{
	return rot;
}
const std::vector<Eigen::Vector2f>& Collider::getWVertices() const
{
	return vertices;
}
const Eigen::Vector2f& Collider::getWVertice(int index) const
{
	return vertices[index];
}
const std::vector<Eigen::Vector2f>& Collider::getWNormals() const
{
	return normals;
}
const Eigen::Vector2f& Collider::getWNormal(int index) const
{
	return normals[index];
}

const AABB& Collider::getAABB() const
{
	return range;
}
const std::vector<std::unique_ptr<Proxy>>& Collider::getProxies() const
{
	return proxies;
}
RigidBody* Collider::getRB() const
{
	return body;
}

void Collider::update()
{
	AABB temp = proxies[0]->getAABB();
	range.x_min = temp.x_min;//x轴坐标
	range.x_max = temp.x_max;
	range.y_min = temp.y_min;//y轴坐标
	range.y_max = temp.y_max;

	for (int i = 1; i < p_size; i++)
	{
		temp = proxies[i]->getAABB();
		range.x_min = std::min(temp.x_min, range.x_min);
		range.x_max = std::max(temp.x_max, range.x_max);
		range.y_min = std::min(temp.y_min, range.y_min);
		range.y_max = std::max(temp.y_max, range.y_max);
	}

	Vec2View lver = shape->getLVertices();
	//Vec2View lnorm = shape->getLNormals();
	for (int i = 0; i < count; i++)
	{
		vertices[i] = Matrix::rotate(transform->rotation) * *(lver.vertice + i) + transform->position;
		//normals[i] = Matrix::rotate(transform->rotation) * *(lnorm.vertice + i);
	}

	rot = Matrix::rotate(transform->rotation);
	for (int i = 0; i < p_size; i++)
	{
		proxies[i]->update();
	}
}

void Collider::addProxy(std::unique_ptr<Proxy> p)
{
	proxies.push_back(std::move(p));
	p_size = proxies.size();
}






Proxy::Proxy(const Collider* par, const Shape* sh, const Eigen::Vector2f& pos, float rot):
parent(par), shape(sh), offsetPos(pos), offsetRot(rot) {}



PolygonProxy::PolygonProxy(const Collider* par, const Shape* sh, const Eigen::Vector2f& pos, float rot) :
	Proxy(par, sh, pos, rot) {}

const Eigen::Vector2f& PolygonProxy::getPos() const
{
	return parent->getPos() + offsetPos;
}
float PolygonProxy::getRot() const
{
	return offsetRot;
}

Eigen::Vector2f PolygonProxy::getSupport(const Eigen::Vector2f& dir) const
{
	Eigen::Vector2f localDir = RT * dir;
	Eigen::Vector2f localSupport = shape->getSupport(localDir);
	return R * localSupport + T;
}

//找到形状在屏幕坐标系下的方形范围，用于粗筛
const AABB& PolygonProxy::getAABB() const
{
	return range;
}

//norm向形状外
Edge PolygonProxy::getBestEdge(const Eigen::Vector2f& norm) const
{
	Eigen::Vector2f localNorm = RT * norm;
	Edge edge = shape->getBestEdge(localNorm);
	edge.transform(R, T);
	return edge;
}

const Collider* PolygonProxy::getCollider() const
{
	return parent;
}

void PolygonProxy::update()
{
	R = parent->getRotMat() * Matrix::rotate(offsetRot);
	RT = R.transpose();
	T = parent->getRotMat() * offsetPos + parent->getPos();
	range = shape->getAABB(T, R);
}
