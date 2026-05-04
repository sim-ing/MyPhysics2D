#include"Solver.h"
#include"Matrix.h"
#include"test.h"
#include<iostream>
#include<cmath>
#include<Eigen/Dense>







//Solver
Solver::Solver(float t,
	const std::vector<ContactManifold>& c, 
	const std::vector<std::shared_ptr<Joint>>& j) :
	dt(t), contacts(c), joints(j)
{
	constraints.reserve(contacts.size() + joints.size());
}

void Solver::presolve()
{
	for (auto& it : constraints)
	{
		it->presolve();
	}
}

void Solver::iterate()
{
	//std::cout << constraints << std::endl;
	state = true;
	int iter = 0;
	while (state && (iter < maxIter))
	{
		//std::cout << "solver iterate:" << iter << std::endl;
		state = false;
		for (auto& it : constraints)
		{
			if (std::abs(it->solve()) > eps)
			{
				state = true;
			}
		}
		iter++;
	}
	//std::cout << "solver iterate:" << iter <<"\n"<< std::endl;
	//std::cout << "constrain zahl:" << constraints.size() << std::endl;
}

void Solver::createConstraints()
{
	for (const auto& contact : contacts)
	{
		for (const auto& point : contact.points)
		{
			bool doFind = false;
			
			//warm starting
			auto cc = cache.find(point.feature);
			if (cc != cache.end())
			{
				for (auto& cp : cc->second)
				{
					//找到了对应的缓存
					if ((cp.midPos - (point.conPosA + point.conPosB) * 0.5f).norm() < 0.1f)
					{
						doFind = true;
						constraints.push_back(std::make_unique<ContactConstraint>(dt,
							contact.colA->getCollider()->getRB(),
							contact.colB->getCollider()->getRB(), point, 
							cp.accumNImpuls, cp.accumTImpuls));
						//std::cout << "erfolg" << std::endl;
						break;
					}
				}
			}
			if (!doFind)
			{
				constraints.push_back(std::make_unique<ContactConstraint>(dt,
					contact.colA->getCollider()->getRB(),
					contact.colB->getCollider()->getRB(), point));
			}
		}




		
	}
	for (const auto& joint : joints)
	{
		constraints.push_back(std::make_unique<JointConstraint>(dt, *joint));
	}
}

void Solver::clearConstraints()
{
	cache.clear();
	for (auto& it : constraints)
	{
		if (auto* c = dynamic_cast<ContactConstraint*>(it.get()))
		{
			cache[c->getFeature()].push_back(c->getCache());
		}
		if (auto* c = dynamic_cast<JointConstraint*>(it.get()))
		{
			c->setCache();
		}
	}
	constraints.clear();
}


