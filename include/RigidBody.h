#pragma once
#include<memory>
#include<array>
#include<vector>
#include<Eigen/Core>
#include"Shape.h"
//#include"Collider.h"



struct Transform
{
	Eigen::Vector2f position;
	float rotation = 0; //单位：弧度制
};

class Collider;

class Joint;


class RigidBody
{
public:
	enum class Type { Dynamik, Statik };
private:
	//从shape到世界坐标的变换信息
	Transform transform;
	float mass;
	float invMass;
	float inertia;
	float invInertia;
	Eigen::Vector2f velocity;
	float angVelocity = 0; //单位：弧度每秒
	Eigen::Vector2f accelerate;
	float angAccelerate = 0;
	std::shared_ptr<Shape> shape; //刚体持有的形状类型
	Type type = Type::Dynamik;

	std::vector<Collider*> colliders;
	std::vector<Joint*> joints;

public:
	
	
	RigidBody(const std::shared_ptr<Shape>& ptr, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero());
	
	//所有getter
	const Transform* getTrans() const;
	Eigen::Vector2f getPos() const;
	float getRot() const;
	float getMass() const;
	float getInvMass() const;
	float getInert() const;
	float getInvInert() const;
	const Eigen::Vector2f& getVel() const;
	float getAngVel() const;
	const Eigen::Vector2f& getAcc() const;
	float getAngAcc() const;
	const std::shared_ptr<Shape>& getShape() const;
	const Type& getType() const;
	const std::vector<Collider*>& getColliders() const;
	const std::vector<Joint*>& getJoints() const;

	//所有setter
	void setPos(const Eigen::Vector2f& pos);
	void setRot(const float& rot);
	void setMass(const float& m);
	void setVel(const Eigen::Vector2f& vel);
	void setAngVel(const float& angVel);
	void setShape(std::shared_ptr<Shape> s);
	void setType(const Type& t);
	void addCollider(Collider* c);
	void addJoint(Joint* j);
	

	//力学交互
	void addAccelerate(const Eigen::Vector2f& acc);
	void addForce(const Eigen::Vector2f& Force);
	void addTorch(const float& Torch);
	//在某点上作用冲量
	void addImplus(const Eigen::Vector2f& pos, const Eigen::Vector2f& implus);
	void clearForce();
	void clearTorch();

	//积分器
	void integrate(const float& dt);
};

