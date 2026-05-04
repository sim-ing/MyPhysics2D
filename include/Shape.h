#pragma once
#include<Eigen/Core>
#include<vector>
#include<array>
#include<ostream>


struct Edge
{
	//A一定在左边
	Eigen::Vector2f pointA, pointB, normal, perp;//向外的法向量,A->B的切向量
	int id;//边的引索，方便后续碰撞管理用

	Edge(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& norm, int index);

	void transform(const Eigen::Matrix2f& R, const Eigen::Vector2f& T);
};

struct AABB
{
	//单个刚体的上下限
	float x_min, x_max, y_min, y_max;

};
bool AABB_overlap(const AABB& rA, const AABB& rB);

struct Vec2View
{
private:
	int size;

public:
	Vec2View(const Eigen::Vector2f* ptr, int c);
	const Eigen::Vector2f* vertice;
	int count()const;
};

//调整多边形顺序的算法，一定产出逆时针多边形点集
std::vector<Eigen::Vector2f> monotonChain(const std::vector<Eigen::Vector2f> vertices);

//计算所有凹凸多边形面积的算法，ver逆时针为正，顺时针为负
float calArea(const std::vector<Eigen::Vector2f>& ver);

//转动惯量
Eigen::Matrix2f calInertia(const std::vector<Eigen::Vector2f>& ver);

//找到形心,ver必须逆时针
Eigen::Vector2f calCenter(const std::vector<Eigen::Vector2f>& ver, float Area);


//用于做earcuting的顶点结构
struct Vertice
{
	//链表结构
	int prev = 0;
	int index = 0;
	int next = 0;

	//描述是否为凸节点，凸节点为真
	bool isConvex = true;
};

//做earcut的算法类
class EarCut
{
private:
	//顶点链表，只有引索
	std::vector<Vertice> nodes;
	//凹顶点向量
	std::vector<int> concaveNodes;
	//原数据
	const std::vector<Eigen::Vector2f>& vertices;
	//结果
	std::vector<Vertice> triangles;

public:
	//只接受逆时针旋转的顶点
	EarCut(const std::vector<Eigen::Vector2f>& ver);
	void deleteVertice(int index);
	bool isEar(int index);
	void cut();
	const std::vector<Vertice>& getTriangles()const;

};




class Proxy;
class Collider;

class Shape
{
public:
	//多态折构函数
	virtual ~Shape() = default;
	virtual float getArea() const = 0;
	virtual Eigen::Matrix2f getInertia() const = 0;

	//创建proxy的方法
	virtual void createProxy(Collider* col,
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0) const = 0;
	
	//找到某个方向上面某形状最远的点，用于细筛
	virtual Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const = 0;

	virtual Edge getBestEdge(const Eigen::Vector2f& norm) const = 0;

	virtual AABB getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const = 0;

	//只对三角形和多边形实现，圆形报错
	virtual Vec2View getLVertices() const = 0;

	virtual const Eigen::Vector2f& getLVertice(int index) const = 0;

	//默认为向外的法向量
	virtual Vec2View getLNormals() const = 0;

	virtual const Eigen::Vector2f& getLNormal(int index) const = 0;
};

class Triangle : public Shape
{
private:
	//三个顶点相对于形心的坐标位置
	std::array<Eigen::Vector2f, 3> vertices;
	//三个顶点对应顶点加一连成边的法向量信息,朝外
	std::array<Eigen::Vector2f, 3> normals;
	

public:
	//默认构造函数，三点必须是以形心为原点的
	Triangle(const std::array<Eigen::Vector2f, 3>& ver);

	//通过三个点来确定三角形的静态函数，同时反馈形心位置给rigidbody
	static Triangle createFromPoints(const std::array<Eigen::Vector2f, 3>& ver, Eigen::Vector2f* cen = nullptr);

	//未来计划更多的特殊三角形构造方法

	float getArea() const override;
	Eigen::Matrix2f getInertia() const override;

	void createProxy(Collider* col,
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0) const override;
	Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const override;
	Edge getBestEdge(const Eigen::Vector2f& norm) const override;
	AABB getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const override;

	Vec2View getLVertices() const override;
	const Eigen::Vector2f& getLVertice(int index) const override;

	//得到vertice和下一点的normal，归一化的
	Vec2View getLNormals() const override;
	const Eigen::Vector2f& getLNormal(int index) const override;
};

class Polygon : public Shape
{
private:
	//顶点相对于形心的坐标位置
	std::vector<Eigen::Vector2f> vertices;
	//顶点对应顶点加一连成边的法向量信息,朝外
	std::vector<Eigen::Vector2f> normals;
	//顶点数量
	int size = 3;

	//标志可信来源的tag
	struct TrustSource {};
	Polygon(const std::vector<Eigen::Vector2f>& ver, TrustSource);

public:
	Polygon(const std::vector<Eigen::Vector2f>& ver);
	//通过点来确定三角形的静态函数，同时反馈形心位置给rigidbody
	static Polygon createFromPoints(const std::vector<Eigen::Vector2f>& ver, Eigen::Vector2f* cen = nullptr);


	float getArea() const override;
	Eigen::Matrix2f getInertia() const override;

	void createProxy(Collider* col, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0) const override;
	Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const override;
	Edge getBestEdge(const Eigen::Vector2f& norm) const override;
	AABB getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const override;
	Vec2View getLVertices() const override;
	const Eigen::Vector2f& getLVertice(int index) const override;
	Vec2View getLNormals() const override;
	const Eigen::Vector2f& getLNormal(int index) const override;
};

class Circle : public Shape
{
private:
	float radius = 0;

public:
	Circle(const float& rad);

	//通过拖拽来实现圆形构造，同时反馈形心位置给rigidbody
	static Circle createFromDrag(const std::array<Eigen::Vector2f, 2>& drag, Eigen::Vector2f& cen);


	float getArea() const override;
	Eigen::Matrix2f getInertia() const override;

	void createProxy(Collider* col, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0) const override;
	Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const override;
	Edge getBestEdge(const Eigen::Vector2f& norm) const override;
	AABB getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const override;
	Vec2View getLVertices() const override;
	const Eigen::Vector2f& getLVertice(int index) const override;
	Vec2View getLNormals() const override;
	const Eigen::Vector2f& getLNormal(int index) const override;
};

class Compound : public Shape
{
private:
	std::vector<Eigen::Vector2f> vertices;

	//组成组合形状的子形状,后续使用一般只依托于child
	struct Child
	{
		std::shared_ptr<Shape> shape;
		Eigen::Vector2f position;
		float rotation = 0;

		Child(const std::shared_ptr<Shape>& sptr);
	};
	std::vector<Child> children;

	int ver_size = 0, chl_size = 0;

	//标志可信来源的tag
	struct TrustSource {};
	Compound(const std::vector<Eigen::Vector2f>& ver, const std::vector<Child>& chl, TrustSource);

public:
	//一定确定以原点为中心
	Compound(const std::vector<Eigen::Vector2f>& ver);
	//通过点来确定三角形的静态函数，同时反馈形心位置给rigidbody
	static Compound createFromPoints(const std::vector<Eigen::Vector2f>& ver, Eigen::Vector2f* cen = nullptr);


	float getArea() const override;
	Eigen::Matrix2f getInertia() const override;

	void createProxy(Collider* col, 
		const Eigen::Vector2f& pos = Eigen::Vector2f::Zero(), float rot = 0) const override;

	//下面的都不会用到，只用子形状的这些函数
	Eigen::Vector2f getSupport(const Eigen::Vector2f& dir) const override;
	Edge getBestEdge(const Eigen::Vector2f& norm) const override;
	AABB getAABB(const Eigen::Vector2f& pos, const Eigen::Matrix2f& rot) const override;
	Vec2View getLVertices() const override;
	const Eigen::Vector2f& getLVertice(int index) const override;
	Vec2View getLNormals() const override;
	const Eigen::Vector2f& getLNormal(int index) const override;
};