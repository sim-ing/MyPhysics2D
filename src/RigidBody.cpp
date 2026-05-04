#include"RigidBody.h"
#include"Joint.h"
#include"Matrix.h"
#include<iostream>
#include<algorithm>
#include<Eigen/Dense>







RigidBody::RigidBody(const std::shared_ptr<Shape>& ptr, const Eigen::Vector2f& pos): shape(ptr)
{ 
	transform.position = pos;
	velocity << 0, 0;
	accelerate << 0, 0;
	mass = 1000;
	invMass = 1 / mass;
	Eigen::Matrix2f temp = shape->getInertia();
	inertia = (temp(0, 0) + temp(1, 1)) * mass / shape->getArea();
	invInertia = 1 / inertia;
}

//所有getter
const Transform* RigidBody::getTrans() const
{
	return &transform;
}
Eigen::Vector2f RigidBody::getPos() const
{
	return transform.position;
}
float RigidBody::getRot() const
{
	return transform.rotation;
}
float RigidBody::getMass() const
{
	return mass;
}
float RigidBody::getInvMass() const
{
	return invMass;
}
float RigidBody::getInert() const
{
	return inertia;
}
float RigidBody::getInvInert() const
{
	return invInertia;
}
const Eigen::Vector2f& RigidBody::getVel() const
{
	return velocity;
}
float RigidBody::getAngVel() const
{
	return angVelocity;
}
const Eigen::Vector2f& RigidBody::getAcc() const
{
	return accelerate;
}
float RigidBody::getAngAcc() const
{
	return angAccelerate;
}
const std::shared_ptr<Shape>& RigidBody::getShape() const
{
	return shape;
}
const RigidBody::Type& RigidBody::getType() const
{
	return type;
}
const std::vector<Collider*>& RigidBody::getColliders() const
{
	return colliders;
}
const std::vector<Joint*>& RigidBody::getJoints() const
{
	return joints;
}






//所有setter
void RigidBody::setPos(const Eigen::Vector2f& pos)
{
	transform.position = pos;
}
void RigidBody::setRot(const float& rot)
{
	transform.rotation = rot;
}
void RigidBody::setMass(const float& m)
{
	if (type == Type::Statik)
	{
		throw "Statik has no mass";
	}
	mass = m;
	invMass = 1 / m;
	Eigen::Matrix2f temp = shape->getInertia();
	inertia = (temp(0, 0) + temp(1, 1)) * m / shape->getArea();
	invInertia = 1 / inertia;
}
void RigidBody::setVel(const Eigen::Vector2f& vel)
{
	velocity = vel;
}
void RigidBody::setAngVel(const float& angVel)
{
	angVelocity = angVel;
}
void RigidBody::setShape(std::shared_ptr<Shape> s)
{
	shape = s;
	Eigen::Matrix2f temp = shape->getInertia();
	inertia = (temp(0, 0) + temp(1, 1)) * mass / shape->getArea();
	invInertia = 1 / inertia;
}
void RigidBody::setType(const Type& t)
{
	switch (t)
	{
	case Type::Statik:
		type = Type::Statik;
		invMass = 0;
		invInertia = 0;
		velocity = Eigen::Vector2f::Zero();
		angVelocity = 0;
		accelerate = Eigen::Vector2f::Zero();
		angAccelerate = 0;
		break;
	case Type::Dynamik:
		type = Type::Dynamik;
		break;
	}
}
void RigidBody::addCollider(Collider* c)
{
	colliders.push_back(c);
}
void RigidBody::addJoint(Joint* j)
{
	joints.push_back(j);
}


//力学交互
void RigidBody::addAccelerate(const Eigen::Vector2f& acc)
{
	accelerate = acc;
}
void RigidBody::addForce(const Eigen::Vector2f& Force)
{
	accelerate += Force * invMass;
}
void RigidBody::addTorch(const float& Torch)
{
	angAccelerate += Torch * invInertia;
}
void RigidBody::addImplus(const Eigen::Vector2f& pos, const Eigen::Vector2f& implus)
{
	velocity += invMass * implus;
	angVelocity += invInertia * math2D::cross(pos, implus);
}
void RigidBody::clearForce()
{
	accelerate = Eigen::Vector2f::Zero();
}
void RigidBody::clearTorch()
{
	angAccelerate = 0;
}

void RigidBody::integrate(const float& dt)
{
	//v_1 = v_0 + a_0 * dt
	velocity += accelerate * dt;
	//x_1 = x_0 + v_1 * dt
	transform.position += velocity * dt;

	//同上
	angVelocity += angAccelerate * dt;
	transform.rotation += angVelocity * dt;
}


