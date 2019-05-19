#pragma once

#include <vector>

class UnionFind
{
public:

    void init(size_t N);

    void makeUnion(size_t i, size_t j);

    void getConnectedComponents(size_t& num_components, std::vector<size_t>& components);

protected:

    void find(size_t i);

protected:

    std::vector<size_t> mAncestor;
};

