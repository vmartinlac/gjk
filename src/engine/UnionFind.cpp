#include "UnionFind.h"

void UnionFind::init(size_t N)
{
    mAncestor.resize(N);
    for(size_t i=0; i<N; i++)
    {
        mAncestor[i] = i;
    }
}

void UnionFind::makeUnion(size_t i, size_t j)
{
    find(i);
    find(j);
    mAncestor[mAncestor[i]] = mAncestor[j];
}

void UnionFind::getConnectedComponents(size_t& num_components, std::vector<size_t>& components)
{
    const size_t N = mAncestor.size();

    num_components = 0;
    components.resize(N);

    for(size_t i=0; i<N; i++)
    {
        find(i);

        if(mAncestor[i] == i)
        {
            components[i] = num_components;
            num_components++;
        }
    }

    for(size_t i=0; i<N; i++)
    {
        if(mAncestor[i] != i)
        {
            components[i] = components[mAncestor[i]];
        }
    }
}

void UnionFind::find(size_t i)
{
    while( mAncestor[i] != mAncestor[mAncestor[i]] )
    {
        mAncestor[i] = mAncestor[mAncestor[i]];
    }
}

