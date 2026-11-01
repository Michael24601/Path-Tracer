
#include "image/imageUtil.hpp"
#include "math/vector3.hpp"
#include "math/constants.hpp"
#include "logger.hpp"

namespace pathtracer{

    namespace ImageUtil{

        real relativeMse(const std::vector<std::vector<Vector3>>& image,
            const std::vector<std::vector<Vector3>>& reference) {

            if(reference.empty()){
                LOG_WARNING("No reference image was included.");
                return INFINITY;
            }

            if(image.size() != reference.size()
                || image[0].size() != reference[0].size()){

                LOG_WARNING("Reference size does not match render size.");
                return INFINITY;
            }

            real sum = 0.0;
            real epsilon = 0.01;

            for(size_t j = 0; j < image.size(); j++){
                for(size_t i = 0; i < image[j].size(); i++){

                    // The reference has values between 0 and 1,
                    // so to compare them, we must first clamp
                    // the renderer result.
                    Vector3 value(
                        std::min(image[j][i].x(), real(1.0)),
                        std::min(image[j][i].y(), real(1.0)),
                        std::min(image[j][i].z(), real(1.0))
                    );

                    Vector3 ref = reference[j][i];
                    Vector3 difference = value - ref;

                    sum += (difference.x() * difference.x()) / (ref.x() * ref.x() + epsilon)
                        + (difference.y() * difference.y()) / (ref.y() * ref.y() + epsilon)
                        + (difference.z() * difference.z()) / (ref.z() * ref.z() + epsilon);
                }
            }

            // Mean over all pixels and all 3 channels
            real count = 3.0 * image.size() * image[0].size();

            return sum / count;
        }
        

    }
}