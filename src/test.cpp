#include "test.h"


std::ostream& operator<<(std::ostream& os, const AABB& obj)
{
	os << "AABB:\n" << obj.x_max <<" "<< obj.x_min << "\n" << obj.y_max <<" "<< obj.y_min << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Edge& obj)
{
	os << "Edge:\nA = \n" << obj.pointA << "\nB = \n" << obj.pointB << "\nn = \n" << obj.normal<<"\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const RefEdge& obj)
{
	os << obj.edge;
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<Eigen::Vector2f>& obj)
{
	size_t i = 0;
	for (auto& it : obj)
	{
		os << "point[" << i << "] :\n" << it << "\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const ContactPair& obj)
{
	//os << "A: " << obj.aIndex << " B:" << obj.bIndex << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<ContactPair>& obj)
{
	os << "pairs.size:" << obj.size()<<"\n";
	for (auto& it : obj)
	{
		os << it;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const AxisPoint& obj)
{
	//os<< "Point:\nId = " << obj.index << " isMin: " << obj.isMin << "\nvalue = " << obj.value << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const SupportPoint& obj)
{
	os << "support point: \n" << obj.point << 
		"\nfrom A: \n" << obj.supportA << 
		"\nfrom B: \n" << obj.supportB <<"\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::array<SupportPoint, 3>& obj)
{
	size_t i = 0;
	for (auto& it : obj)
	{
		os << "simplex["<<i<<"] :\n"<< it.point<<"\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<SupportPoint>& obj)
{
	size_t i = 0;
	for (auto& it : obj)
	{
		os << "polytope[" << i << "] :\n" << it<< "\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const ContactPoint& obj)
{
	os << "contactpoint A:\n" << obj.conPosA << "\ncontactpoint B:\n" << obj.conPosB
		<< "\nnorm:\n" << obj.conNorm << "\npenetration: " << obj.penetration << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const ContactManifold& obj)
{
	//os << "index: " << obj.aIndex << "    " << obj.bIndex << "\n";
	size_t i = 0;
	for (auto& it : obj.points)
	{
		os << "point[" << i << "] :\n" << it << "\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<ContactManifold>& obj)
{
	size_t i = 0;
	for (auto& it : obj)
	{
		os << "contact[" << i << "] :\n" << it << "\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const ContactConstraint& obj)
{
	os << "exist";
	return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<ContactConstraint>& obj)
{
	size_t i = 0;
	for (auto& it : obj)
	{
		os << "constraint[" << i << "] : " << it << "\n";
		i++;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const RigidBody::Type& obj)
{
	switch (obj)
	{
	case RigidBody::Type::Dynamik:
		os << "Dynamik";
		break;
	case RigidBody::Type::Statik:
		os << "Statik";
		break;
	}
	return os;
}
