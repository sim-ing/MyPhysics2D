#pragma once
#include<memory>
#include<vector>
#include<array>
#include"RigidBody.h"
#include"Collider.h"




//AABB出来的有潜力相撞的刚体对,只能由AABB修改
struct ContactPair
{
	//刚体引索
	//int aIndex, bIndex;
	const Collider* cA;
	const Collider* cB;
	//表示y轴是否有重叠
	//false表示无重叠
	//bool yOverlap;
};

struct PairKey
{
	//int aIndex, bIndex;
	const Collider* cA;
	const Collider* cB;

	PairKey(const Collider* A, const Collider* B);

	bool operator==(const PairKey& o) const;
};

struct PairHash
{
	size_t operator()(const PairKey& k)const;
};

//x轴缓存数据
struct AxisPoint
{
	bool isMin;
	float value;
	//int index;
	const Collider* collider;

};




//AABB管理员
class PairManager
{
private:
	//const std::vector<PolygonCollider>& bodies;

	//从小到达排列
	std::vector<AxisPoint> xAxis;

	std::unordered_map<PairKey, ContactPair, PairHash> pairs;

public:
	PairManager();

	void updateData();

	void updatePairs();

	void addPoint(const Collider* col);

	void deletePoint(const Collider* col);

	void deletePoints(const std::vector<const Collider*>& cols);

	void getPairsVector(std::vector<ContactPair>& res);

private:
	void createContact(const Collider* A, const Collider* B);

	void deleteContact(const Collider* A, const Collider* B);

};