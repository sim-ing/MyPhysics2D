#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Collider.h"
#include"Shape.h"
#include"Pairs.h"
#include"Contact.h"
#include"Collision.h"
#include"Joint.h"
#include"Constraint.h"
#include"Solver.h"
#include<Eigen/Core>




//处理碰撞的类，持有所有刚体
class Physics
{
private:
	//所有刚体
	std::vector<std::unique_ptr<RigidBody>> bodies;
	
	//所有关节
	std::vector<std::shared_ptr<Joint>> joints;

	//刚体碰撞接口
	std::vector<std::shared_ptr<Collider>> colliders;

	//所有碰撞对
	std::vector<ContactManifold> contacts;


	PairManager pairManager;

	//AABB得到的粗筛可能碰撞对
	std::vector<ContactPair> pairs;

	//迭代求解器
	Solver solver;

	//时间的离散
	float dt = 0.01;

	bool autoTest = true;
	

public:
	Physics();

	//粗筛碰撞对
	void detPosColl();

	//找到真实碰撞，GJK
	//找到碰撞的深度和法向
	void detTruColl();

	//计算碰撞结果，解LCP/QP
	void calColl();

	//删除超出画面的刚体
	void delDead();

	//时间向前一步,
	void step();

	//创造刚体
	void createRB(const Eigen::Vector2f& center);

	//删除实体
	void deleteRB(int index);

	//删除碰撞对
	void deleteCon(int index);

	//void addJoint();

	//getter
	const std::shared_ptr<Collider>& getCol(int index);
	bool aT()const;

	int getBodySize()const;
};





