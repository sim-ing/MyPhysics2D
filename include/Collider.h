#pragma once
#include<memory>
#include<array>
#include<vector>
#include<Eigen/Core>
#include"Shape.h"
#include"RigidBody.h"




//用于表示rigidbody的不同材质和属性，负责管理cache
class Collider
{
private:
	//形状的信息,只储存最小子形
	const std::shared_ptr<Shape> shape;
	std::vector<std::unique_ptr<Proxy>> proxies;
	//proxy/分形个数
	int count, p_size;

	//刚体的信息
	const Transform* transform;
	//提供给solver用，自己不可使用
	RigidBody* body;

	//旋转变换矩阵的缓存
	Eigen::Matrix2f rot;
	AABB range;

	//世界坐标的物理量缓存，以后render的成员
	std::vector<Eigen::Vector2f> vertices;
	std::vector<Eigen::Vector2f> normals;

public:
	Collider(RigidBody* b);

	const Eigen::Vector2f& getPos() const;
	float getRot() const;
	const Eigen::Matrix2f& getRotMat() const;

	//之后renderer的成员
	const std::vector<Eigen::Vector2f>& getWVertices() const;
	const Eigen::Vector2f& getWVertice(int index) const;
	const std::vector<Eigen::Vector2f>& getWNormals() const;
	const Eigen::Vector2f& getWNormal(int index) const;


	//找到形状在屏幕坐标系下的方形范围，用于粗筛
	const AABB& getAABB() const;
	const std::vector<std::unique_ptr<Proxy>>& getProxies() const;
	RigidBody* getRB() const;

	void update();
	void addProxy(std::unique_ptr<Proxy> p);
};





//单个collider对于Collision算法的接口
class Proxy
{
protected:
	const Collider* parent;
	const Shape* shape;

	Eigen::Vector2f offsetPos = Eigen::Vector2f::Zero();
	float offsetRot = 0;

	//Proxy的world transform的一份缓存，会包含offset，
	//由collider集中更新管理
	Eigen::Matrix2f R, RT;
	Eigen::Vector2f T;

	AABB range;

public:
	Proxy(const Collider* par, const Shape* sh, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0);

	virtual ~Proxy() = default;

	virtual const Eigen::Vector2f& getPos() const = 0;
	virtual float getRot() const = 0;

	virtual Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const = 0;
	//找到形状在屏幕坐标系下的方形范围，用于粗筛
	virtual const AABB& getAABB() const = 0;
	//norm向形状外
	virtual Edge getBestEdge(const Eigen::Vector2f& norm) const = 0;
	virtual const Collider* getCollider() const = 0;

	//用于更新缓存的世界坐标变换
	virtual void update() = 0;
};

class PolygonProxy : public Proxy
{
private:
	//多边形有几条边
	int count;

public:
	PolygonProxy(const Collider* par, const Shape* sh, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0);

	const Eigen::Vector2f& getPos() const override;
	float getRot() const override;
	Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const override;
	//找到形状在屏幕坐标系下的方形范围，用于粗筛
	const AABB& getAABB() const override;
	//norm向形状外
	Edge getBestEdge(const Eigen::Vector2f& norm) const override;
	const Collider* getCollider() const override;

	void update() override;
};
