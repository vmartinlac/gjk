#include "LinkedBody.h"

template<int D>
LinkPtr<D> LinkedBody<D>::getLink()
{
    return mLinkToParent;
}

template<int D>
void LinkedBody<D>::setLink(const LinkPtr<D>& link)
{
    mLinkToParent = link;
}

template<int D>
std::vector<BodyPtr<D>>& LinkedBody<D>::refChildren()
{
    return mChildren;
}

template class LinkedBody<2>;
template class LinkedBody<3>;

