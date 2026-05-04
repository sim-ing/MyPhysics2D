#include"Pairs.h"
#include"test.h"
#include<cmath>


PairKey::PairKey(const Collider* A, const Collider* B):cA(A), cB(B)
{
	//if (aId < bId) { aIndex = aId; bIndex = bId; }
	//else { aIndex = bId; bIndex = aId; }

}

bool PairKey::operator==(const PairKey& o) const
{
	return (o.cA == cA && o.cB == cB)||(o.cA == cB && o.cB == cA);
}


size_t PairHash::operator()(const PairKey& k)const
{
	return std::hash<const Collider*>()(k.cA) ^ std::hash<const Collider*>()(k.cB);
}




//AABBËã·¨
PairManager::PairManager(){}

void PairManager::updateData()
{
	for (auto& it : xAxis)
	{
		if (it.isMin)
		{
			//it.value = bodies[it.index].getAABB().x_min;
			it.value = it.collider->getAABB().x_min;
		}
		else
		{
			//it.value = bodies[it.index].getAABB().x_max;
			it.value = it.collider->getAABB().x_max;
		}
	}
}

void PairManager::updatePairs()
{
	for (int i = 1; i < xAxis.size(); i++)
	{
		AxisPoint temp = xAxis[i];
		for (int j = i; j > 0; j--)
		{
			AxisPoint next = xAxis[j - 1];

			if (temp.value <= next.value)
			{
				if (temp.isMin && !next.isMin)
				{
					createContact(temp.collider, next.collider);
				}
				if (!temp.isMin && next.isMin)
				{
					deleteContact(temp.collider, next.collider);
				}

				std::swap(xAxis[j], xAxis[j - 1]);
			}
			else
			{
				break;
			}
		}
	}
}

void PairManager::addPoint(const Collider* col)
{
	AxisPoint newMin, newMax;
	newMin.isMin = true;
	newMax.isMin = false;
	//newMin.value = col->getAABB().x_min;
	//newMax.value = col->getAABB().x_max;
	newMin.collider = col;
	newMax.collider = col;
	xAxis.push_back(newMin);
	xAxis.push_back(newMax);
}

void PairManager::deletePoint(const Collider* col)
{
	std::erase_if(xAxis, [&](const AxisPoint& it) {return it.collider == col; });

	auto it = pairs.begin();
	while (it != pairs.end())
	{
		if (it->first.cA == col || it->first.cB == col)
		{
			it = pairs.erase(it);
		}
		else
		{
			it++;
		}
	}
}

void PairManager::deletePoints(const std::vector<const Collider*>& cols)
{
	for (auto& it : cols)
	{
		deletePoint(it);
	}
}

void PairManager::createContact(const Collider* A, const Collider* B)
{
	PairKey key(A, B);
	pairs[key] = ContactPair(A, B);
}

void PairManager::deleteContact(const Collider* A, const Collider* B)
{
	PairKey key(A, B);
	pairs.erase(key);
}

void PairManager::getPairsVector(std::vector<ContactPair>& res)
{
	res.clear();

	for (auto& it : pairs)
	{
		//float aMax = bodies[it.second.aIndex].getAABB().y_max;
		//float bMin = bodies[it.second.bIndex].getAABB().y_min;
		//float bMax = bodies[it.second.bIndex].getAABB().y_max;

		float aMin = it.second.cA->getAABB().y_min;
		float aMax = it.second.cA->getAABB().y_max;
		float bMin = it.second.cB->getAABB().y_min;
		float bMax = it.second.cB->getAABB().y_max;

		bool aba1 = (aMin <= bMin) && (aMax >= bMin);
		bool aba2 = (aMin <= bMax) && (aMax >= bMax);
		if (aba1 || aba2)
		{
			//it.second.yOverlap = true;
			res.push_back(it.second);
		}
	}
}
