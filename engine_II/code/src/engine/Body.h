
#pragma once

#include <memory>
#include "BodyModel.h"
#include "Link.h"

template<int D>
class Body;

template<int D>
using BodyPtr = std::shared_ptr<Body<D>>;

template<int D>
class Body
{
public:

    void setId(size_t id);

    size_t getId();

    LinkPtr<D> getLink();

protected:

    size_t mId;
    BodyModelPtr<D> mModel;
    LinkPtr<D> mLinkToParent;
    std::vector<BodyPtr<D>> mChildren;
};


