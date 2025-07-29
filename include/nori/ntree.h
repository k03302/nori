/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/common.h>

NORI_NAMESPACE_BEGIN

constexpr unsigned int power(unsigned int base, unsigned int exp) {
	return (exp <= 0) ? 1 : base * power(base, exp - 1);
}

constexpr bool isPowerOfTwo(int x) {
	return x > 0 && (x & (x - 1)) == 0;
}

constexpr unsigned int log2(unsigned int x){
	return (x <= 1) ? 0 : 1 + log2(x >> 1);
}


struct ValidationData
{
	ValidationData() : m_isValid(false) {}
	bool m_isValid;
};



template <typename NodeType, unsigned int ChildCount, unsigned int Depth>
class NTree
{
public:
	NTree() : m_count(0) {
		static_assert(std::is_base_of<ValidationData, NodeType>::value, "NodeType must inherit from BaseNode");
		static_assert(isPowerOfTwo(ChildCount));
		static_assert(nodesAvailable < std::numeric_limits<int>::max());
	}

	const NodeType& getNode(int nodeIndex)
	{
		if (nodeIndex < 0 || nodeIndex >= nodesAvailable) {
			throw std::out_of_range("Node index out of range");
		}
		return m_nodes[nodeIndex];
	}

	static int getParentNode(int nodeIndex)
	{
		if (nodeIndex < 0 || nodeIndex >= nodesAvailable) {
			throw std::out_of_range("Node index out of range");
		}
		return (nodeIndex - 1) / ChildCount;
	}

	static int getChildNode(int parentIndex, int childIndex)
	{
		if (parentIndex < 0 || parentIndex >= nodesAvailable) {
			throw std::out_of_range("Parent index out of range");
		}
		if (childIndex < 0 || childIndex >= ChildCount) {
			throw std::out_of_range("Child index out of range");
		}
		return parentIndex * ChildCount + childIndex + 1;
	}

public:
	static constexpr int nodesAvailable = power(ChildCount, Depth);
	static constexpr int shiftCount = log2(ChildCount);
	NodeType m_nodes[nodesAvailable];
	int m_count;
};

NORI_NAMESPACE_END
