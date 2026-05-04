#include"Contact.h"
#include"Matrix.h"
#include"test.h"
#include<iostream>
#include<cmath>
#include<Eigen/Dense>

Feature::Feature(const Proxy* pA, const Proxy* pB) :
	bodyA(pA->getCollider()->getRB()), bodyB(pB->getCollider()->getRB()){}

bool Feature::operator ==(const Feature& f) const
{
	return (bodyA == f.bodyA && bodyB == f.bodyB) || (bodyA == f.bodyB && bodyB == f.bodyA);
}

size_t FeatureHash::operator()(const Feature& f) const
{
	return std::hash<const RigidBody*>()(f.bodyA) ^ std::hash<const RigidBody*>()(f.bodyB);
}


RefEdge::RefEdge(const Edge& e, const Belong& from) :edge(e), belong(from) {}


ContactPoint::ContactPoint(const Feature& f):feature(f){}

ContactPoint ContactPoint::createFromRefA(const Feature& feature, const Eigen::Vector2f& incP, const Edge& ref, const Eigen::Vector2f& norm)
{
	ContactPoint temp(feature);
	temp.conNorm = norm;
	temp.conPosB = incP;
	temp.conPosA = math2D::getProjection(incP, ref.pointA, ref.perp);
	temp.penetration = (temp.conPosA - temp.conPosB).dot(norm.normalized());
	return temp;
}
ContactPoint ContactPoint::createFromRefB(const Feature& feature, const Eigen::Vector2f& incP, const Edge& ref, const Eigen::Vector2f& norm)
{
	
	ContactPoint temp(feature);
	temp.conNorm = norm;
	temp.conPosA = incP;
	temp.conPosB = math2D::getProjection(incP, ref.pointA, ref.perp);
	temp.penetration = (temp.conPosA - temp.conPosB).dot(norm.normalized());
	return temp;
}


ContactManifold::ContactManifold(const Proxy* A, const Proxy* B) : colA(A), colB(B)
{
	points.reserve(2);
}



//ContactManage
ContactManager::ContactManager(const Proxy* A, const Proxy* B) :colA(A), colB(B) {}

SupportPoint ContactManager::getSupport(const Eigen::Vector2f& dir)
{
	SupportPoint result;
	result.supportA = colA->getSupport(dir);
	result.supportB = colB->getSupport(-dir);
	result.point = result.supportA - result.supportB;
	return result;
}

void ContactManager::detectContact()
{
	GJK gjk;
	gjk.step(getSupport(colB->getPos() - colA->getPos()));
	while (gjk.getState() == GJK::State::Continue && gjkIter < maxGJK)
	{
		++gjkIter;
		gjk.step(getSupport(gjk.getNextNorm()));
	}

	if (gjk.getState() == GJK::State::Nointersect)
	{
		doCon = false;
		//std::cout << ">>>>>>no intersect!!!!!!!!<<<<<<\n" << std::endl;
	}
	else
	{
		//std::cout << "<<<<<<intersect!!!!!!!!>>>>>>\n" << std::endl;
		doCon = true;
		EPA epa(gjk.getSimplex());

		while (!epa.getState() && epaIter < maxEPA)
		{
			epaIter++;
			epa.step(getSupport(epa.getNextNorm()));
		}

		if (!epa.getState())
		{
			// EPA 未收敛，保守处理：视为无有效深度/法线
			std::cout << "EPA failed" << std::endl;
		}

		conDepth = epa.getDepth();
		conNorm = epa.getNextNorm();

		
		//std::cout << "\n\n<<<<<epa end here>>>>>\ndepth: " << conDepth << "\nnorm:\n" << conNorm << std::endl;
	}
}

ContactManifold ContactManager::generateContact()
{
	ContactManifold manifold(colA, colB);
	RefEdge ref = getReference();
	RefEdge inc = getIncident(ref);
	std::vector<Eigen::Vector2f> points = clip(ref, inc);
	Feature feature(colA, colB);

	if (inc.belong == RefEdge::Belong::A)
	{
		for (auto& it : points)
		{
			manifold.points.push_back(ContactPoint::createFromRefB(feature, it, ref.edge, conNorm));
		}
	}
	else
	{
		for (auto& it : points)
		{
			manifold.points.push_back(ContactPoint::createFromRefA(feature, it, ref.edge, conNorm));
		}
	}
	

	return manifold;
}

RefEdge ContactManager::getReference() const
{
	Edge edgeA = colA->getBestEdge(conNorm);
	Edge edgeB = colB->getBestEdge(-conNorm);

	float productA = edgeA.normal.dot(conNorm);
	float productB = edgeB.normal.dot(-conNorm);

	//更贴合A
	if (productA > productB)
	{
		return RefEdge(edgeA, RefEdge::Belong::A);
	}
	else
	{
		return RefEdge(edgeB, RefEdge::Belong::B);
	}
}

RefEdge ContactManager::getIncident(const RefEdge& ref) const
{
	switch (ref.belong)
	{
	case RefEdge::Belong::A:
		return RefEdge(colB->getBestEdge(-ref.edge.normal), RefEdge::Belong::B);
	case RefEdge::Belong::B:
		return RefEdge(colA->getBestEdge(-ref.edge.normal), RefEdge::Belong::A);
	}
}

std::vector<Eigen::Vector2f> ContactManager::clip(const RefEdge& ref, const RefEdge& inc) const
{
	std::vector<Eigen::Vector2f> res(2);
	res[0] = inc.edge.pointA;
	res[1] = inc.edge.pointB;
	float ldA, ldB, rdA, rdB, udA, udB;

	//ref左切
	ldA = math2D::getPLDistance(inc.edge.pointA, ref.edge.pointA, ref.edge.normal) + eps;
	ldB = math2D::getPLDistance(inc.edge.pointB, ref.edge.pointA, ref.edge.normal) + eps;
	//ref右切
	rdA = math2D::getPLDistance(inc.edge.pointA, ref.edge.pointB, ref.edge.normal) - eps;
	rdB = math2D::getPLDistance(inc.edge.pointB, ref.edge.pointB, ref.edge.normal) - eps;
	//ref下切
	udA = math2D::getPLDistance(inc.edge.pointA, ref.edge.pointB, ref.edge.perp) + eps;
	udB = math2D::getPLDistance(inc.edge.pointB, ref.edge.pointB, ref.edge.perp) + eps;

	//都在左边
	if (ldA < 0 && ldB < 0)
	{
		res.clear();
		return res;
	}
	//一左一右
	else if (ldA < 0 && ldB >= 0)
	{
		res[0] = math2D::getIntersectPoint(
			inc.edge.pointA, inc.edge.perp,
			ref.edge.pointA, ref.edge.normal);
	}
	else if (ldA >= 0 && ldB < 0)
	{
		res[1] = math2D::getIntersectPoint(
			inc.edge.pointA, inc.edge.perp,
			ref.edge.pointA, ref.edge.normal);
	}

	//都在右边
	if (rdA > 0 && rdB > 0)
	{
		res.clear();
		return res;
	}
	//一左一右
	else if (rdA > 0 && rdB <= 0)
	{
		res[0] = math2D::getIntersectPoint(
			inc.edge.pointA, inc.edge.perp,
			ref.edge.pointB, ref.edge.normal);
	}
	else if (rdA <= 0 && rdB > 0)
	{
		res[1] = math2D::getIntersectPoint(
			inc.edge.pointA, inc.edge.perp,
			ref.edge.pointB, ref.edge.normal);
	}

	//全在上面（无交点）
	if (udA < 0 && udB < 0)
	{
		res.clear();
		return res;
	}
	//一上一下
	else if (udA < 0 && udB >= 0)
	{
		res[0] = std::move(res.back());
		res.pop_back();
	}
	else if (udA >= 0 && udB < 0)
	{
		res[1] = std::move(res.back());
		res.pop_back();
	}

	return res;
}

bool ContactManager::getIntersection() const
{
	return doCon;
}

bool ContactManager::jointExist() const
{
	for (const auto& it : colA->getCollider()->getRB()->getJoints())
	{
		if (it->getA() == colA->getCollider()->getRB() && 
			it->getB() == colB->getCollider()->getRB() ||
			it->getB() == colA->getCollider()->getRB() && 
			it->getA() == colB->getCollider()->getRB())
		{
			return true;
		}
	}
	return false;
}
