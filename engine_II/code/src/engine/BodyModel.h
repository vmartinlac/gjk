
#pragma once

#include <memory>

template<int D>
class BodyModel
{
public:

    double getMass();

protected:

};

template<int D>
using BodyModelPtr = std::shared_ptr<BodyModel<D>>;

