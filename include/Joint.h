#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Shape.h"

#include<Eigen/Core>

struct Joint
{
	RigidBody* rbA;
	RigidBody* rbB;

	//碰撞发生点，相对位置
	Eigen::Vector2f posA, posB;

	Eigen::Vector2f cache = Eigen::Vector2f::Zero();

	Joint(RigidBody* a, RigidBody* b, const Eigen::Vector2f& pA, const Eigen::Vector2f& pB);

	Eigen::Vector2f getPos() const;
	const RigidBody* getA() const;
	const RigidBody* getB() const;

};