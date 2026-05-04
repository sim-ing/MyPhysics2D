#pragma once
#include<memory>
#include<vector>
#include<array>
#include<Eigen/Core>


//GJK专用的支撑点数据
//现在不太有用，以后找机会可以删了，但是调试可能还用得上
struct SupportPoint
{
	Eigen::Vector2f point, supportA, supportB;

	SupportPoint();

	SupportPoint(const SupportPoint& other);
};



//GJK算法类
class GJK
{
public:
	enum class State
	{
		Continue,
		Intersect,
		Nointersect
	};

private:
	//float eps = 1e-6;
	std::array<SupportPoint, 3> simplex;
	int simplexCount = 0;

	//下一步迭代前进的方向
	Eigen::Vector2f nextNorm;

	//上一个更新的simplex顶点，原点一定会在这个点的两条边外
	int lastPoint = 2;


	//是否交集
	State state = State::Continue;

public:
	//输入外部给的新suppoertpoint，gjk只要判断过不过原点就可以了
	//返回下一个点的support方向，交给ContactDetect做下一步迭代
	void step(const SupportPoint& newpoint);

	Eigen::Vector2f getNextNorm() const;

	const std::array<SupportPoint, 3>& getSimplex() const;

	State getState() const;

private:
	//判断通过向量norm新找到的点A是否过原点
	//过为真仍有可能相交，不过为假不相交
	bool reachOrigin(const SupportPoint& A, const Eigen::Vector2f& norm) const;

	//数学类中getnorm对于supportpoint的重载
	Eigen::Vector2f getNorm(const SupportPoint& A, const SupportPoint& B) const;

	//判断原点在那个vonoroi外，没找到返回lastpoint
	//其中一个引索确定为lastPoint，即新加入的点
	int inArea(Eigen::Vector2f& norm) const;

};

//EPA算法类
class EPA
{
private:
	float eps = 1e-3;
	std::vector<SupportPoint> polytope;
	int polySize = 3;

	//下一步迭代前进的方向，同时也是conNorm
	Eigen::Vector2f nextNorm;
	//找到最近边的引索，给外部找到supportpoint，给内部插入新的点
	int lastPoint = 0;

	//碰撞深度
	float conDepth = 0;

	//是否找到找到最短边
	bool state = false;

public:
	EPA(const std::array<SupportPoint, 3>& simplex);

	void step(const SupportPoint& newpoint);

	Eigen::Vector2f getNextNorm() const;

	bool getState() const;

	float getDepth() const;

private:
	//加入点
	void addPoint(int index, const SupportPoint& newPoint);
	//替换点
	void repPoint(int index, const SupportPoint& newPoint);

	//原点向外的线段法向量
	Eigen::Vector2f getNorm(int index);

	//找到两个点连成的边到原点的距离
	float getDistance(int index) const;
	//找到最短边
	int findSide();

};