#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Contact.h"
#include"Joint.h"
#include"Constraint.h"
#include<Eigen/Core>




//迭代求解器
class Solver
{
private:
	//std::vector<RigidBody>& bodies;

	const std::vector<ContactManifold>& contacts;
	const std::vector<std::shared_ptr<Joint>>& joints;

	//只有solver持有的约束类
	//std::vector<cachedContact> cache;
	std::unordered_map<Feature, std::vector<CachedPoint>, FeatureHash> cache;
	std::vector<std::unique_ptr<Constraint>> constraints;

	bool state = true;

	float dt = 0.01;
	float eps = 1e-3;
	int maxIter = 75;

public:
	Solver(float t, const std::vector<ContactManifold>& c, const std::vector<std::shared_ptr<Joint>>& j);

	void presolve();

	void iterate();

	void createConstraints();

	void clearConstraints();

};