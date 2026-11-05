
#include "image/imageUtil.hpp"
#include "math/vector3.hpp"
#include "math/constants.hpp"
#include <algorithm>
#include "logger.hpp"

namespace pathtracer{

    namespace ImageUtil{


        real mse(const std::vector<std::vector<Vector3>>& image,
            const std::vector<std::vector<Vector3>>& reference) {

            if(reference.empty() || image.empty()){
                LOG_WARNING("Image or reference image is empty.");
                return INFINITY;
            }

            if(image.size() != reference.size()
                || image[0].size() != reference[0].size()){

                LOG_WARNING("Reference size does not match render size.");
                return INFINITY;
            }

            real sum = 0.0;

            for(int i = 0; i < image.size(); i++){
                for(int j = 0; j < image[i].size(); j++){

                    Vector3 ref = Vector3(
                        std::clamp(reference[i][j].x(), 0.0, 50.0),
                        std::clamp(reference[i][j].y(), 0.0, 50.0),
                        std::clamp(reference[i][j].z(), 0.0, 50.0)
                    );
                    Vector3 im = Vector3(
                        std::clamp(image[i][j].x(), 0.0, 50.0),
                        std::clamp(image[i][j].y(), 0.0, 50.0),
                        std::clamp(image[i][j].z(), 0.0, 50.0)
                    );
                    Vector3 difference = im - ref;

                    sum += difference.x() * difference.x()
                        + difference.y() * difference.y()
                        + difference.z() * difference.z();
                }
            }

            real count = 3.0 * image.size() * image[0].size();

            return sum / count;
        }


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
            constexpr real epsilon = 0.01;

            for(int i = 0; i < image.size(); i++){
                for(int j = 0; j < image[i].size(); j++){

                    // No need to clamp the rendered output here because
                    // we assume the reference image uses HDR format.

                    Vector3 ref = Vector3(
                        std::clamp(reference[i][j].x(), 0.0, 50.0),
                        std::clamp(reference[i][j].y(), 0.0, 50.0),
                        std::clamp(reference[i][j].z(), 0.0, 50.0)
                    );
                    Vector3 im = Vector3(
                        std::clamp(image[i][j].x(), 0.0, 50.0),
                        std::clamp(image[i][j].y(), 0.0, 50.0),
                        std::clamp(image[i][j].z(), 0.0, 50.0)
                    );
                    Vector3 difference = im - ref;

                    sum += (difference.x() * difference.x()) / (ref.x() * ref.x() + epsilon)
                        + (difference.y() * difference.y()) / (ref.y() * ref.y() + epsilon)
                        + (difference.z() * difference.z()) / (ref.z() * ref.z() + epsilon);
                }
            }

            // Mean over all pixels and all 3 channels
            real count = 3.0 * image.size() * image[0].size();

            return sum / count;
        }
        

        Vector3 meanColor(const std::vector<std::vector<Vector3>>& image) {

            if(image.empty() || image[0].empty()){
                LOG_WARNING("Image is empty.");
                return Vector3(NAN);
            }

            Vector3 sum(0.0, 0.0, 0.0);
            int count = image[0].size() * image.size();

            for(const auto& row : image){
                for(const Vector3& pixel : row){
                    sum = sum + pixel;
                }
            }

            return sum / static_cast<real>(count);
        }

    }
}