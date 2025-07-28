#include <nori/ntree.h>
#include <iostream>
using namespace nori;

class OctNode : public ValidationData
{
};

int main()
{
    NTree<OctNode, 8, 3> tree;
    std::cout << tree.shiftCount;
    return 0;
}