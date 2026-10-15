
#ifndef PATH_TRACER_KERNEL_HPP
#define PATH_TRACER_KERNEL_HPP

#include "config.hpp"

namespace pathtracer{


    enum class KernelType{
        UNIFORM,
        EPANECHNIKOV
    };


    namespace Kernel{

        // Each kernel returns the weight of a point given the distance
        // and radius.
        real uniformWeight(real distanceSquared, real radiusSquared);

        real epanechnikovWeight(real distanceSquared, real radiusSquared);

        real weight(KernelType type, real distanceSquared, real radiusSquared);

    };

}

#endif