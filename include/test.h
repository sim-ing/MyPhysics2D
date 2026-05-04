#pragma once

#include<Eigen/Core>
#include<ostream>
#include"Shape.h"
#include"RigidBody.h"
#include"Physics.h"
#include"Collision.h"
#include"Contact.h"
#include"Solver.h"



std::ostream& operator<<(std::ostream& os, const AABB& obj);

std::ostream& operator<<(std::ostream& os, const Edge& obj);

std::ostream& operator<<(std::ostream& os, const RefEdge& obj);

std::ostream& operator<<(std::ostream& os, const std::vector<Eigen::Vector2f>& obj);

std::ostream& operator<<(std::ostream& os, const ContactPair& obj);

std::ostream& operator<<(std::ostream& os, const std::vector<ContactPair>& obj);

std::ostream& operator<<(std::ostream& os, const AxisPoint& obj);

std::ostream& operator<<(std::ostream& os, const SupportPoint& obj);

std::ostream& operator<<(std::ostream& os, const std::array<SupportPoint, 3>& obj);

std::ostream& operator<<(std::ostream& os, const std::vector<SupportPoint>& obj);

std::ostream& operator<<(std::ostream& os, const ContactPoint& obj);

std::ostream& operator<<(std::ostream& os, const ContactManifold& obj);

std::ostream& operator<<(std::ostream& os, const std::vector<ContactManifold>& obj);

std::ostream& operator<<(std::ostream& os, const ContactConstraint& obj);

std::ostream& operator<<(std::ostream& os, const std::vector<ContactConstraint>& obj);

std::ostream& operator<<(std::ostream& os, const RigidBody::Type& obj);