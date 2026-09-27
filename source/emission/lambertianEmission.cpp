
#include "emission/lambertianEmission.hpp"
#include "math/vector2.hpp"
#include "math/mathUtil.hpp"

namespace pathtracer{

    LambertianEmission::LambertianEmission(
        const Vector3& emissionColor) :
        m_emissionColor(emissionColor) {}

    Vector3 LambertianEmission::evaluate(
        const Vector3& wo, const Vector2& uv) const {

        // The emission in wo is weighted by the cosine of the
        // angle the normal makes with the outgoing ray of light.
        real cosine = ShadingSpace::cosineTheta(wo);

        if(cosine <= 0) {
            return Vector3::ORIGIN;
        }

        return m_emissionColor;
    }

}