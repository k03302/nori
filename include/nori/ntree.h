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

template <typename NodeType, unsigned int ChildCount, unsigned int Depth> class NTree;

template <typename NodeType, unsigned int ChildCount, unsigned int Depth>
class NTreeVisitor
{
public:
	NTreeVisitor(NTree<NodeType, ChildCount, Depth>* ntree)
		: m_tree(ntree), m_depth(0), m_nodeIndex(0)
	{
		static_assert(std::is_base_of<ValidationData, NodeType>::value, "NodeType must inherit from BaseNode");
		static_assert(isPowerOfTwo(ChildCount));
		static_assert(nodesAvailable < std::numeric_limits<int>::max());
	}

	void visitParent()
	{
		if (m_depth == 0) {
			throw std::out_of_range("Already at root, cannot visit parent");
		}
		m_nodeIndex = (m_nodeIndex - 1) / ChildCount;
	}

	void visitChild(int childIndex)
	{
		if (m_depth >= Depth || childIndex < 0 || childIndex >= ChildCount) {
			throw std::out_of_range("Invalid child index or depth exceeded");
		}
		m_nodeIndex = m_nodeIndex * ChildCount + childIndex;
		m_depth++;
	}



private:
	NTree<NodeType, ChildCount, Depth>* m_tree;
	static constexpr int shiftCount = log2(ChildCount);
	int m_depth;
	int m_nodeIndex;
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

	

public:
	static constexpr int nodesAvailable = power(ChildCount, Depth);
	static constexpr int shiftCount = log2(ChildCount);
	NodeType m_nodes[nodesAvailable];
	int m_count;
};

NORI_NAMESPACE_END
