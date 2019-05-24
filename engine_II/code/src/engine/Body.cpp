#include "Body.h"

template<int D>
void Body<D>::setId(size_t id)
{
    mId = id;
}

template<int D>
size_t Body<D>::getId()
{
    return mId;
}

template<int D>
LinkPtr<D> Body<D>::getLink()
{
    return mLinkToParent;
}

template class Body<2>;
template class Body<3>;

