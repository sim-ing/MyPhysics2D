#include"Joint.h"
#include"Matrix.h"



Joint::Joint(RigidBody* a, RigidBody* b, const Eigen::Vector2f& pA, const Eigen::Vector2f& pB):
	rbA(a), rbB(b), posA(pA), posB(pB){ }

Eigen::Vector2f Joint::getPos() const
{
	return Matrix::rotate(rbA->getRot()) * posA + rbA->getPos();
}

const RigidBody* Joint::getA() const
{
	return rbA;
}

const RigidBody* Joint::getB() const
{
	return rbB;
}