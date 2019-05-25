#pragma once

#include "Body.h"

template<int D>
class LinkedBody : public Body<D>
{
public:

    LinkPtr<D> getLink();

    void setLink(const LinkPtr<D>& link);

    std::vector<BodyPtr<D>>& refChildren();

private:

    LinkPtr<D> mLinkToParent;
    std::vector<BodyPtr<D>> mChildren;
};

template<int D>
using LinkedBodyPtr = std::shared_ptr< LinkedBody<D> >;

