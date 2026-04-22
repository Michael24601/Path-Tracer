

#ifndef PATH_TRACER_TRANSFORM_HPP
#define PATH_TRACER_TRANSFORM_HPP

#include "../math/vector3.hpp"
#include "../math/matrix3.hpp"
#include "../math/matrix4.hpp"
#include "../core/ray.hpp"

namespace pathtracer{

    // Forward declaration
    class SurfacePoint;

    class Transform{

    private:
        
        // Scaling and rotation
        Matrix3 m_transform;
        // Translation
        Vector3 m_translation;

    public:

        static Transform IDENTITY;
    
        // Identity matrix and no translation
        Transform(){
            m_transform = Matrix3::IDENTITY;
            m_translation = Vector3::ORIGIN;
        }


        Transform(const Vector3& angles, const Vector3& scale, 
            const Vector3& translation) : m_translation(translation) {

            for(int i = 0; i <= 2; i++){
                m_transform.addRotation(angles[i], i);
            }
            for(int i = 0; i <= 2; i++){
                m_transform.scale(scale[i], i);
            }
        }


        Transform(const Matrix3& transform, const Vector3& translation) : 
            m_translation(translation), m_transform(transform) {}


        Transform inverse() const{
            Matrix3 invM = m_transform.inverse();
            Vector3 invT = -(invM * m_translation);
            return Transform(invM, invT);
        }


        // Returns the inverse tranpose without the translation
        Transform inverseTranspose() const{
            Matrix3 invM = m_transform.inverse().transposed();
            Vector3 t = Vector3::ORIGIN;
            return Transform(invM, t);
        }


        // Transforms a point
        Vector3 transform(const Vector3& point) const{
            return (m_transform * point) + m_translation;
        }


        const Matrix3& transform() const{
            return m_transform;
        }


        const Vector3& translation() const{
            return m_translation;
        }


        // Transforms a point
        Vector3 inverseTransform(const Vector3& point) const{
            Matrix3 m = m_transform.inverse();
            return (m * point) - (m * m_translation);
        }


        // Transforms a direction (no translation, must remain normal)
        Vector3 transformDirection(const Vector3& direction) const{
            return (m_transform * direction).normalized();
        }


        // Transforms a direction
        Vector3 inverseTransformDirection(const Vector3& direction) const{
            Matrix3 m = m_transform.inverse();
            return (m * direction).normalized();
        }


        // Transforms a normal (must remain orthogonal),
        // so we use the inverse transposed
        Vector3 transformNormal(const Vector3& normal) const{
            return (m_transform.inverse().transposed() * normal).normalized();
        }


        // Inverse transforms a normal (from world to local)
        Vector3 inverseTransformNormal(const Vector3& normal) const {
            return (m_transform.transposed() * normal).normalized();
        }

        
        // Transforms a ray
        Ray transform(const Ray& ray) const{
            Ray r(transform(ray.origin()), 
                transformDirection(ray.direction()));
            return r;
        }


        Ray inverseTransform(const Ray& ray) const{
            Ray r(inverseTransform(ray.origin()), 
                inverseTransformDirection(ray.direction()));
            return r;
        }


        // Transforms surface points (like intersections)
        SurfacePoint transformSurfacePoint(const SurfacePoint& it) const;


        // Returns the determinant of the transform matrix
        real determinant() const{
            return m_transform.determinant();
        }


        std::string toString() const {
            return "Transform(\n" 
                + m_transform.toString() + "\n"
                + m_translation.toString() + "\n)";
        }
       
    };

}

#endif