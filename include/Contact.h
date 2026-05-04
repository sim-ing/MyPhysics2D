#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Shape.h"
#include"Collision.h"
#include"Collider.h"
#include<Eigen/Core>




struct Feature
{
	const RigidBody* bodyA;
	const RigidBody* bodyB;

	Feature(const Proxy* pA, const Proxy* pB);

	bool operator ==(const Feature& f) const;
};

struct FeatureHash
{
	size_t operator()(const Feature& f)const;
};

struct RefEdge
{
	Edge edge;
	enum class Belong { A, B };
	Belong belong;

	RefEdge(const Edge& e, const Belong& from);
};


//碰撞物体之间接触的点
struct ContactPoint
{
	Feature feature;
	Eigen::Vector2f conPosA, conPosB;
	//A的conNorm方向
	Eigen::Vector2f conNorm;
	float penetration;

	ContactPoint(const Feature& f);

	//inc的点，ref的边，还有个碰撞法向量
	static ContactPoint createFromRefA(const Feature& feature, const Eigen::Vector2f& incP, const Edge& ref, const Eigen::Vector2f& norm);
	static ContactPoint createFromRefB(const Feature& feature, const Eigen::Vector2f& incP, const Edge& ref, const Eigen::Vector2f& norm);
};

//两个物体的碰撞具体点
struct ContactManifold
{
	//持有两个刚体的引索
	//int aIndex, bIndex;
	const Proxy* colA;
	const Proxy* colB;
	std::vector<ContactPoint> points;

	ContactManifold(const Proxy* A, const Proxy* B);

};


//找具体碰撞专用类：GJK，EPA，生成碰撞
class ContactManager
{
private:
	//暂时先用rigidbody，可以考虑把需要的形状和位置消息再打包一个新的结构出来，就不让manager直接持有刚体了
	const Proxy* colA;
	const Proxy* colB;
	//int aIndex, bIndex;

	bool doCon = false;
	float conDepth;
	Eigen::Vector2f conNorm;//向外

	float eps = 1e-5;
	const int maxGJK = 20;
	const int maxEPA = 42;
	int gjkIter = 0, epaIter = 0;

public:
	ContactManager(const Proxy* A, const Proxy* B);

	void detectContact();

	ContactManifold generateContact();

	//getter
	bool getIntersection() const;

	bool jointExist() const;

private:
	//找到对应方向support点的函数
	SupportPoint getSupport(const Eigen::Vector2f& dir);

	//给generator用的妙妙工具
	RefEdge getReference() const;
	RefEdge getIncident(const RefEdge& ref) const;

	std::vector<Eigen::Vector2f> clip(const RefEdge& ref, const RefEdge& inv) const;

	
};