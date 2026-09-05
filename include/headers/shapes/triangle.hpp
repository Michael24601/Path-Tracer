
#ifndef PATH_TRACER_TRIANGLE_HPP
#define PATH_TRACER_TRIANGLE_HPP

#include "shape.hpp"
#include "../math/constants.hpp"
#include "../intersection/IntersectionList.hpp"
#include "../intersection/areaSample.hpp"

namespace pathtracer{

    class Triangle : public Shape{

    private:

        // The triangle is stored using 3 vertices in global
        // coordinates, along with 3 normals that may or may not be
        // given.
        Vector3 v0, v1, v2;
        Vector3 n0, n1, n2;
        bool m_shadingNormals;

        
        // Given a ray and a distance t along it, sets the intersection
        // object. The uv are the barycentric coordinates of the
        // intersected point.
        Intersection generateIntersection(const Ray& ray, real t, 
            const Vector2& uv) const {

            Vector3 point = ray.at(t);

            // The shading and geometry normal are not the same,
            // unless no vertex normals are given.
            Vector3 geometryNormal = ((v1-v0).cross(v2-v0)).normalized();
            Vector3 shadingNormal = m_shadingNormals 
                ? Barycentric::interpolate(n0, n1, n2, uv)
                : geometryNormal;

            // Tangent calculation
            Vector3 tangent = (v1 - v0).normalized();
            tangent = (tangent - shadingNormal * 
                shadingNormal.dot(tangent)).normalized();

            return Intersection(t, point, geometryNormal, 
                shadingNormal, tangent, uv, nullptr);
        }


    public:

        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2) : 
            v0{v0}, v1{v1}, v2{v2}, m_shadingNormals{false} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }


        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector3& n0, const Vector3& n1, const Vector3& n2) : 
            v0{v0}, v1{v1}, v2{v2}, n0{n0}, n1{n1}, n2{n2}, 
            m_shadingNormals{true} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }



        real getSurfaceArea() const override{
            Vector3 r0 = v1 - v0;
            Vector3 r1 = v2 - v0;
            return (r0.cross(r1)).length();
        }


        AxisAlignedBox getBoundingBox() const override{
            Vector3 minimumPoint = v0.min(v1.min(v2));
            Vector3 maximumPoint = v0.max(v1.max(v2));
            return AxisAlignedBox(minimumPoint, maximumPoint);
        }

        
        // Intersects the shape with a ray
        void intersect(const Ray& ray, IntersectionList& list) const override{
            // We can set the parametric ray equation equal to
            // the parametric plane equation, get an intersection,
            // and then ensure it is inside the triangle
            // (called the Möller-Trumbore algorithm).

            Vector3 origin = ray.origin();
            Vector3 edge0 = v1 - v0;
            Vector3 edge1 = v2 - v0;

            // Precomputed for efficiency
            Vector3 rayE1Cross = ray.direction().cross(edge1);
            Vector3 originE0Cross = (origin - v0).cross(edge0);

            float det = edge0.dot(rayE1Cross);

            // It's better not to use == with floating point numbers
            if(abs(det) < EPSILON) return;

            float invDet = 1.0f / det;
            
            float detu = (origin - v0).dot(rayE1Cross);
            float u = detu * invDet;
            if(u < 0.0f || u > 1.0f) return;

            float detv = ray.direction().dot(originE0Cross);
            float v = detv * invDet;
            if(v < 0.0f || u + v > 1.0f) return;

            float dett = edge1.dot(originE0Cross);
            float t = dett * invDet;

            // Here we can conclude we have an intersection
            Vector2 uv(u, v);
            list.push(generateIntersection(ray, t, uv));
        }

            
        AreaSample sampleSurfaceArea() const override{

            Vector2 random = Random::next2D();

            real sqrtU = sqrtReal(random.x());

            real b0 = 1.0 - sqrtU;
            real b1 = sqrtU * (1.0 - random.y());
            real b2 = sqrtU * random.y();

            Vector2 uv(b1, b2);

            Vector3 point = Barycentric::interpolate(v0, v1, v2, uv);

            real pdf = 1.0 / getSurfaceArea();

            return AreaSample(
                generateIntersection(
                    Ray(point, Vector3(0, 0, 1)),
                    0,
                    uv
                ),
                pdf
            );
        }


        AreaSample evaluateAreaSample(const Vector3& point) const override{

            Vector3 edge0 = v1 - v0;
            Vector3 edge1 = v2 - v0;
            Vector3 relative = point - v0;

            real d00 = edge0.dot(edge0);
            real d01 = edge0.dot(edge1);
            real d11 = edge1.dot(edge1);
            real d20 = relative.dot(edge0);
            real d21 = relative.dot(edge1);

            real denominator = d00 * d11 - d01 * d01;

            real u = (d11 * d20 - d01 * d21) / denominator;
            real v = (d00 * d21 - d01 * d20) / denominator;

            Vector2 uv(u, v);

            real pdf = 1.0 / getSurfaceArea();

            return AreaSample(
                generateIntersection(
                    Ray(point, Vector3(0, 0, 1)),
                    0,
                    uv
                ),
                pdf
            );
        }

    };

}

#endif