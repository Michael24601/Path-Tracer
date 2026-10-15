
#include "photonMapping/kernel.hpp"
#include <cassert>
#include "math/constants.hpp"
#include "logger.hpp"

namespace pathtracer{

    namespace Kernel{

        real uniformWeight(real distanceSquared, real radiusSquared){
            if(distanceSquared <= radiusSquared) {
                return 1.0;
            }
            return 0.0;
        }

        real epanechnikovWeight(real distanceSquared, real radiusSquared){
            if(distanceSquared > radiusSquared) {
                return 0.0;
            }

            return 2.0 * (1.0 - distanceSquared / radiusSquared);
        }

        real weight(KernelType type, real distanceSquared, real radiusSquared){

            assert(distanceSquared >= 0 && radiusSquared > 0);

            switch(type){
                case KernelType::UNIFORM: 
                    return uniformWeight(distanceSquared, radiusSquared);
                case KernelType::EPANECHNIKOV:
                    return epanechnikovWeight(distanceSquared, radiusSquared);
            }

            LOG_ERROR("Kernel type is unknown");
            exit(1);
        }

    }

}