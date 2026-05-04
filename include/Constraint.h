#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Contact.h"
#include"Joint.h"
#include<Eigen/Core>


struct CachedPoint
{
	Eigen::Vector2f midPos = Eigen::Vector2f::Zero();

	float accumNImpuls = 0;
	float accumTImpuls = 0;

};




class Constraint
{
public:
	virtual void presolve() = 0;
	virtual float solve() = 0;
};


//碰撞约束，碰撞方程管理
class ContactConstraint : public Constraint
{
private:
	RigidBody* rbA;
	RigidBody* rbB;
	//碰撞发生点，相对位置（世界坐标下相对于质心，已经经过旋转操作）
	Eigen::Vector2f posA, posB;

	Eigen::Vector2f norm; //碰撞法向量，A的
	Eigen::Vector2f perp; //切向量，A的

	float friction = 0.2;

	//等效质量，J*M^(-1)*J
	float effMassN;
	float effMassT;

	//转动项对应的jacobian
	float JnA, JnB;
	float JtA, JtB;
	//速度项的jacobian，A为-norm，B为norm

	//累计施加的冲量，后面计算摩擦要用
	float accumNImpuls = 0;
	float accumTImpuls = 0;

	float e = 0.1;
	float slop = 0.00001;
	//相对速度小于该值不处理弹性碰撞
	float doBance = 0.01;

	//弹性系数，分离物体之类都在其中,一个修正项
	//迭代完应该得到多大的冲量，这里应该小于等于0
	float bias;

	Feature feature;

public:
	ContactConstraint(float dt, RigidBody* A, RigidBody* B, const ContactPoint& contact, float N = 0.0f, float T = 0.0f);

	const Feature& getFeature() const;
	CachedPoint getCache() const;

	void presolve() override;
	//一次求解,返回相对速度进展，过小时由外部solver判断迭代完成
	float solve() override;

private:
	//返回相对速度：J*dq/dt
	float getRelNVel() const;
	float getRelTVel() const;

};

class JointConstraint : public Constraint
{
private:
	RigidBody* rbA;
	RigidBody* rbB;
	Joint* joint;
	//铰链链接点在原形状中的相对位置，局部坐标
	Eigen::Vector2f posA, posB;
	Eigen::Vector2f wPosA, wPosB; //旋转过后链接点位置

	//等效质量，J*M^(-1)*J
	Eigen::Matrix2f effMass;
	Eigen::Matrix2f invEffMass;
	//转动项对应的jacobian
	Eigen::Vector2f JiA, JiB;

	Eigen::Vector2f accumImpuls = Eigen::Vector2f::Zero();

	Eigen::Vector2f bias = {0, 0};

public:
	JointConstraint(float dt, Joint& j);

	void presolve() override;
	float solve() override;

	//这里职责出现混乱，后续所有这类管理全都由physics做，先这么干了
	void setCache();

private:
	//返回相对速度：J*dq/dt
	Eigen::Vector2f getRelVel() const;

	Eigen::Vector2f getRelPos() const;

};
