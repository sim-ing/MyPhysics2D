#include"Constraint.h"
#include"Matrix.h"
#include"test.h"
#include<iostream>
#include<cmath>
#include<Eigen/Dense>


ContactConstraint::ContactConstraint(float dt, RigidBody* A, RigidBody* B, const ContactPoint& contact, float N, float T) :
	rbA(A), rbB(B), norm(contact.conNorm.normalized()), feature(contact.feature),
	accumNImpuls(N), accumTImpuls(T)
{
	//std::cout << "\nN: " << accumNImpuls << "\nT: " << accumTImpuls << std::endl;
	perp = Eigen::Vector2f(-norm.y(), norm.x());
	posA = contact.conPosA - rbA->getPos();
	posB = contact.conPosB - rbB->getPos();

	JnA = math2D::cross(posA, norm);
	JnB = math2D::cross(posB, norm);
	effMassN =
		rbA->getInvMass() +
		rbA->getInvInert() * JnA * JnA +
		rbB->getInvMass() +
		rbB->getInvInert() * JnB * JnB;

	JtA = math2D::cross(posA, perp);
	JtB = math2D::cross(posB, perp);
	effMassT =
		rbA->getInvMass() +
		rbA->getInvInert() * JtA * JtA +
		rbB->getInvMass() +
		rbB->getInvInert() * JtB * JtB;

	//bias = std::min<float>(e * getRelNVel(), 0.0f) - std::max<float>(1 * contact.penetration, 0.0);
	bias = -std::max<float>(2 * contact.penetration, 0);
	if (getRelNVel() < -doBance)
	{
		bias += e * getRelNVel();
	}
	/*
	std::cout
		<< "\nnorm: \n" << norm
		<< "\nposA: \n" << posA
		<< "\nposB: \n" << posB
		<< "\nJiA: " << JiA
		<< "\nJiB: " << JiB
		<< "\nAinvMass: " << rbA.getInvMass()
		<< "\nAinvIntertia: " << rbA.getInvInert()
		<< "\nBinvMass: " << rbB.getInvMass()
		<< "\nBinvIntertia: " << rbB.getInvInert()
		<<"\nbias:"<<bias << std::endl;*/
}

void ContactConstraint::presolve()
{
	rbA->addImplus(posA, -accumNImpuls * norm - accumTImpuls * perp);
	rbB->addImplus(posB, accumNImpuls * norm + accumTImpuls * perp);
	//std::cout << "rel_vel from presolve:" << getRelNVel() << std::endl;
}

float ContactConstraint::solve()
{
	float dNImplus = -(getRelNVel() + bias) / effMassN;
	float dTImplus = -getRelTVel() / effMassT;
	/*std::cout << "\nimplus: " << dNImplus << "\nImplus(accumulated): " << accumNImpuls << std::endl;
	std::cout << "\n<<<<<<before>>>>>>" <<
		"\nvA : \n" << rbA->getVel() <<
		"\nomegaA : " << rbA->getAngVel() <<
		"\nvB : \n" << rbB->getVel() <<
		"\nomegaB : " << rbB->getAngVel() << std::endl;
	std::cout << "rel_vel:" << getRelNVel() << std::endl;*/
	float oldImpuls = accumNImpuls;
	accumNImpuls = std::max<float>(oldImpuls + dNImplus, 0);
	dNImplus = accumNImpuls - oldImpuls;

	float maxTImplus = friction * accumNImpuls;
	float newAccumTImplus = std::clamp<float>(accumTImpuls + dTImplus, -maxTImplus, maxTImplus);
	dTImplus = newAccumTImplus - accumTImpuls;
	accumTImpuls = newAccumTImplus;

	rbA->addImplus(posA, -dNImplus * norm - dTImplus * perp);
	rbB->addImplus(posB, dNImplus * norm + dTImplus * perp);
	/*std::cout << "\n<<<<<<after>>>>>>" <<
		"\nvA : \n" << rbA->getVel() <<
		"\nomegaA : " << rbA->getAngVel() <<
		"\nvB : \n" << rbB->getVel() <<
		"\nomegaB : " << rbB->getAngVel() << std::endl;*/
	return dNImplus * effMassN;
}

float ContactConstraint::getRelNVel() const
{
	return -norm.dot(rbA->getVel()) - JnA * rbA->getAngVel() +
		norm.dot(rbB->getVel()) + JnB * rbB->getAngVel();
}

float ContactConstraint::getRelTVel() const
{
	return -perp.dot(rbA->getVel()) - JtA * rbA->getAngVel() +
		perp.dot(rbB->getVel()) + JtB * rbB->getAngVel();
}




const Feature& ContactConstraint::getFeature() const
{
	return feature;
}

CachedPoint ContactConstraint::getCache() const
{
	return CachedPoint((rbA->getPos() + posA + rbB->getPos() + posB) * 0.5,
		accumNImpuls, accumTImpuls);
}



JointConstraint::JointConstraint(float dt, Joint& j) :
	rbA(j.rbA), rbB(j.rbB), posA(j.posA), posB(j.posB),
	joint(&j), accumImpuls(j.cache)
{
	wPosA = Matrix::rotate(rbA->getRot()) * posA;
	wPosB = Matrix::rotate(rbB->getRot()) * posB;
	JiA = Matrix::rotateAbleit(rbA->getRot()) * posA;
	JiB = Matrix::rotateAbleit(rbB->getRot()) * posB;
	effMass =
		(rbA->getInvMass() + rbB->getInvMass()) * Eigen::Matrix2f::Identity() +
		rbA->getInvInert() * JiA * JiA.transpose() +
		rbB->getInvInert() * JiB * JiB.transpose();
	invEffMass = effMass.inverse();
	//bias = -getRelPos() * 0.001;
}

void JointConstraint::presolve()
{
	rbA->addImplus(wPosA, -accumImpuls);
	rbB->addImplus(wPosB, accumImpuls);
}

float JointConstraint::solve()
{
	Eigen::Vector2f dImpuls = invEffMass * (-getRelVel() - bias);
	accumImpuls = accumImpuls + dImpuls;
	rbA->addImplus(wPosA, -dImpuls);
	rbB->addImplus(wPosB, dImpuls);
	return (effMass * dImpuls).norm();
}

void JointConstraint::setCache()
{
	joint->cache = accumImpuls;
}

Eigen::Vector2f JointConstraint::getRelVel() const
{
	return -rbA->getVel() - rbA->getAngVel() * JiA +
		rbB->getVel() + rbB->getAngVel() * JiB;
}

Eigen::Vector2f JointConstraint::getRelPos() const
{
	return -rbA->getPos() + rbB->getPos();
}