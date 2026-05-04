#include"Physics.h"
#include"Matrix.h"
#include"test.h"
#include<iostream>
#include<cmath>
#include<Eigen/Dense>




//Physics
Physics::Physics():bodies(), colliders(), pairManager(), solver(dt, contacts, joints)
{
	Eigen::Vector2f center;
	Eigen::Vector2f ver1(0, 10);
	Eigen::Vector2f ver2(640, 10);
	Eigen::Vector2f ver3(640, 1);
	Eigen::Vector2f ver4(0, 1);
	Polygon A = Polygon::createFromPoints({ ver1, ver2, ver3, ver4 },&center);
	std::shared_ptr<Shape> sA = std::make_shared<Polygon>(A);

	bodies.push_back(std::make_unique<RigidBody>(sA, center));

	RigidBody* p_rbA = bodies.back().get();
	
	auto colA = std::make_shared<Collider>(p_rbA);
	p_rbA->setType(RigidBody::Type::Statik);
	
	colliders.push_back(colA);
	pairManager.addPoint(colA.get());
	p_rbA->addCollider(colA.get());
}

void Physics::detPosColl()
{
	pairManager.updateData();
	pairManager.updatePairs();
	pairManager.getPairsVector(pairs);
	//std::cout <<"pairs count: " << pairs.size() << std::endl;
}

void Physics::detTruColl()
{
	//std::cout << "\npairs:" << pairs.size() << std::endl;
	for (const auto& it : pairs)
	{
		for (const auto& pA : it.cA->getProxies())
		{
			for (const auto& pB : it.cB->getProxies())
			{
				if (!AABB_overlap(pA->getAABB(), pB->getAABB())) continue;

				ContactManager manager(pA.get(), pB.get());
				manager.detectContact();

				if ((manager.getIntersection() == true) && (!manager.jointExist()))
				{
					contacts.push_back(manager.generateContact());
					autoTest = false;
				}
				else
				{
					autoTest = true;
				}
			}
		}
	}
	//std::cout << "contacts:" << contacts.size() << std::endl;
}

void Physics::calColl()
{
	solver.createConstraints();
	solver.presolve();
	solver.iterate();
	solver.clearConstraints();
}

void Physics::delDead()
{
	//int idex = bodies.size();
	for (int index = 0; index < bodies.size(); index++)
	{
		if (bodies[index]->getPos().squaredNorm() > 1000000)
		{
			deleteRB(index);
		}
	}
}




void Physics::step()
{
	delDead();
	for (auto& it : colliders)
	{
		it->update();
	}
	detPosColl();
	detTruColl();
	calColl();
	//size_t i = 0;
	for (auto& it : bodies)
	{
		//std::cout << "body["<<i<<"]: \n"<<"body v : \n" << it.getVel() << "\nbody w : " << it.getAngVel() << std::endl;
		it->integrate(dt);
		//i++;
	}
	//std::cout << bodies.size() << "\n" << std::endl;
	contacts.clear();
}

void Physics::createRB(const Eigen::Vector2f& center)
{
	Eigen::Vector2f ver1(100, 100);
	Eigen::Vector2f ver2(100, 200);
	Eigen::Vector2f ver3(200, 200);
	Eigen::Vector2f ver4(200, 100);
	Eigen::Vector2f ver5(150, 150);
	Eigen::Vector2f cenA, cenB;
	Triangle A = Triangle::createFromPoints({ ver1, ver2, ver3 }, &cenA);
	Triangle B = Triangle::createFromPoints({ ver3, ver4, ver5 }, &cenB);
	//Triangle A = Triangle::createFromPoints({ ver1, ver2, ver3 });
	std::shared_ptr<Shape> p_A = std::make_shared<Triangle>(A);
	std::shared_ptr<Shape> p_B = std::make_shared<Triangle>(B);

	Eigen::Vector2f offset = cenB - cenA;
	bodies.push_back(std::make_unique<RigidBody>(p_A, center));
	RigidBody* p_rbA = bodies.back().get();
	auto colA = std::make_shared<Collider>(p_rbA);

	bodies.push_back(std::make_unique<RigidBody>(p_B, center + offset));
	RigidBody* p_rbB = bodies.back().get();
	auto colB = std::make_shared<Collider>(p_rbB);

	joints.push_back(std::make_shared<Joint>(p_rbA, p_rbB, ver3 - cenA, ver3 - cenB));
	
	
	colliders.push_back(colA);
	colliders.push_back(colB);
	pairManager.addPoint(colA.get());
	pairManager.addPoint(colB.get());
	p_rbA->addCollider(colA.get());
	p_rbB->addCollider(colB.get());
	p_rbA->addJoint(joints.back().get());
	p_rbB->addJoint(joints.back().get());
	p_rbA->addAccelerate({ 0,-9.81 });
	p_rbB->addAccelerate({ 0,-9.81 });

}

void Physics::deleteRB(int index)
{
	//不对vector里的rb操作，只是让pairmanager忘记他，下次覆盖他
	//bodies[index] = RigidBody();
	//freeBId.push_back(index);
	//pairManager.deletePoint(index);
	const std::vector<Collider*>& colTargets = bodies[index]->getColliders();
	for (auto* it : colTargets)
	{
		pairManager.deletePoint(it);
		std::erase_if(colliders, [&](std::shared_ptr<Collider> c) {return c.get() == it; });
	}

	const std::vector<Joint*>& jointTargets = bodies[index]->getJoints();
	for (auto* it : jointTargets)
	{
		std::erase_if(joints, [&](std::shared_ptr<Joint> c) {return c.get() == it; });
	}
	
	bodies[index] = std::move(bodies.back());
	bodies.pop_back();
}

void Physics::deleteCon(int index)
{
	contacts[index] = std::move(contacts.back());
	contacts.pop_back();
}


const std::shared_ptr<Collider>& Physics::getCol(int index)
{
	return colliders[index];
}
bool Physics::aT() const
{
	return autoTest;
}
int Physics::getBodySize()const
{
	return bodies.size();
}