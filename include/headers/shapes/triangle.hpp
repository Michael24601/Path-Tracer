
#ifndef PATH_TRACER_TRIANGLE_HPP
#define PATH_TRACER_TRIANGLE_HPP

#include "shape.hpp"
#include "../math/constants.hpp"
#include "../intersection/intersection.hpp"
#include "../intersection/areaSample.hpp"
#include "../intersection/surfaceDifferentials.hpp"
#include "../logger.hpp"

namespace pathtracer{

    class Triangle : public Shape{

    private:

        // The triangle is stored using 3 vertices in global
        // coordinates, along with 3 normals that may or may not be
        // given.
        Vector3 v0, v1, v2;
        Vector3 n0, n1, n2;
        // UV coordinates of each triangle vertex
        Vector2 uv0, uv1, uv2;
        bool m_shadingNormals;
        bool m_uvCoordinates;



        Vector3 computeDpdu() const {
            Vector3 e1 = v1 - v0;
            Vector3 e2 = v2 - v0;

            Vector2 duv1, duv2;

            if (m_uvCoordinates) {
                duv1 = uv1 - uv0;
                duv2 = uv2 - uv0;
            } else {
                duv1 = Vector2(1, 0);
                duv2 = Vector2(0, 1);
            }

            real det =
                duv1.x() * duv2.y() -
                duv2.x() * duv1.y();

            if (std::abs(det) < EPSILON)
                return e1;

            real invDet = 1.0 / det;

            return (e1 * duv2.y() - e2 * duv1.y()) * invDet;
        }


        
        // Given a ray and a distance t along it, sets the intersection
        // object. The uv are the barycentric coordinates of the
        // intersected point.
        SurfacePoint generateSurfacePoint(const Ray& ray, real t, 
            const Vector2& barycentric) const {

            Vector3 point = ray.at(t);

            // The shading and geometry normal are not the same,
            // unless no vertex normals are given.
            Vector3 geometryNormal = ((v1-v0).cross(v2-v0)).normalized();
            Vector3 shadingNormal = m_shadingNormals 
                ? Barycentric::interpolate(n0, n1, n2, barycentric).normalized()
                : geometryNormal;

            // Tangent calculation
            Vector3 dpdu = computeDpdu();
            Vector3 tangent =
                (dpdu - shadingNormal * shadingNormal.dot(dpdu)).normalized();

            
            // The uv (texture) coordinates can be specified, otherwise
            // we use the barycentric coordinates of the intersection
            // as a fallback.
            Vector2 uv = m_uvCoordinates
                ? Barycentric::interpolate(uv0, uv1, uv2, barycentric)
                : barycentric;

            SurfacePoint sp(point, geometryNormal, 
                shadingNormal, tangent, uv, nullptr);

            return sp;
        }


    public:

        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2) : 
            v0{v0}, v1{v1}, v2{v2}, m_shadingNormals{false}, m_uvCoordinates{false} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }


        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector3& n0, const Vector3& n1, const Vector3& n2) : 
            v0{v0}, v1{v1}, v2{v2}, n0{n0}, n1{n1}, n2{n2}, 
            m_shadingNormals{true}, m_uvCoordinates{false} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }


        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector2& uv0, const Vector2& uv1, const Vector2& uv2) : 
            v0{v0}, v1{v1}, v2{v2}, uv0{uv0}, uv1{uv1}, uv2{uv2}, 
            m_uvCoordinates{true}, m_shadingNormals{false} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }

        
        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector3& n0, const Vector3& n1, const Vector3& n2,
            const Vector2& uv0, const Vector2& uv1, const Vector2& uv2) : 
            v0{v0}, v1{v1}, v2{v2}, n0{n0}, n1{n1}, n2{n2},
            uv0{uv0}, uv1{uv1}, uv2{uv2}, 
            m_shadingNormals{true}, m_uvCoordinates{true} {

                // Ensures points are not colinear
                Vector3 n = (v1-v0).cross(v2-v0);
                assert ((n.lengthSquared() > EPSILON) && "Degenerate triangle");
            }



        real getSurfaceArea() const override{
            Vector3 r0 = v1 - v0;
            Vector3 r1 = v2 - v0;
            return 0.5 * (r0.cross(r1)).length();
        }


        AxisAlignedBox getBoundingBox() const override{
            Vector3 minimumPoint = v0.min(v1.min(v2));
            Vector3 maximumPoint = v0.max(v1.max(v2));
            return AxisAlignedBox(minimumPoint, maximumPoint);
        }


        Vector3 getCentroid() const override{
            // Just the vertex average
            return (v0 + v1 + v2) * ONE_THIRD;
        }

        
        // Intersects the shape with a ray
        Intersection intersect(const Ray& ray, real oldT) const override{

            Intersection intersection;

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

            real det = edge0.dot(rayE1Cross);

            // It's better not to use == with floating point numbers
            if(abs(det) < EPSILON) return intersection;

            real invDet = 1.0f / det;
            
            real detu = (origin - v0).dot(rayE1Cross);
            real u = detu * invDet;
            if(u < 0.0f || u > 1.0f) return intersection;

            real detv = ray.direction().dot(originE0Cross);
            real v = detv * invDet;
            if(v < 0.0f || u + v > 1.0f) return intersection;

            real dett = edge1.dot(originE0Cross);
            real t = dett * invDet;

            if(t > SHADOW_EPSILON && t < oldT){
                // Here we can conclude we have an intersection
                Vector2 barycentric(u, v);
                return Intersection(t, generateSurfacePoint(ray, t, barycentric));
            }

            return intersection;
        }

            
        AreaSample sampleSurfaceArea() const override{

            Vector2 random = Random::next2D();

            real sqrtU = sqrtReal(random.x());

            real b0 = 1.0 - sqrtU;
            real b1 = sqrtU * (1.0 - random.y());
            real b2 = sqrtU * random.y();

            Vector2 barycentric(b1, b2);
            Vector3 point = Barycentric::interpolate(v0, v1, v2, barycentric);

            real pdf = 1.0 / getSurfaceArea();

            return AreaSample(
                generateSurfacePoint(
                    Ray(point, Vector3(0, 0, 1)),
                    0,
                    barycentric
                ),
                pdf
            );
        }


        AreaSample evaluateAreaSample(const SurfaceSample& point) const override{

            Vector3 edge0 = v1 - v0;
            Vector3 edge1 = v2 - v0;
            Vector3 relative = point.point - v0;

            real d00 = edge0.dot(edge0);
            real d01 = edge0.dot(edge1);
            real d11 = edge1.dot(edge1);
            real d20 = relative.dot(edge0);
            real d21 = relative.dot(edge1);

            real denominator = d00 * d11 - d01 * d01;

            real b0 = (d11 * d20 - d01 * d21) / denominator;
            real b1 = (d00 * d21 - d01 * d20) / denominator;
            Vector2 barycentric(b0, b1);

            real pdf = 1.0 / getSurfaceArea();

            return AreaSample(
                generateSurfacePoint(
                    Ray(point.point, Vector3(0, 0, 1)),
                    0,
                    barycentric
                ),
                pdf
            );
        }


        SurfaceDifferentials computeDifferentials(
            const Vector3& position, const Vector3& shadingNormal,
            const Vector2& uv, int triangleIndex) const override {

            Vector3 e1 = v1 - v0;
            Vector3 e2 = v2 - v0;

            Vector2 duv1, duv2;

            if (m_uvCoordinates) {
                duv1 = uv1 - uv0;
                duv2 = uv2 - uv0;
            } else {
                // Barycentric fallback
                duv1 = Vector2(1, 0);
                duv2 = Vector2(0, 1);
            }

            real det =
                duv1.x() * duv2.y() -
                duv1.y() * duv2.x();

            Vector3 dpdu, dpdv;

            if (std::abs(det) < EPSILON) {
                dpdu = e1;
                dpdv = e2;
            } else {
                real invDet = 1.0 / det;

                dpdu =
                    (e1 * duv2.y() -
                    e2 * duv1.y()) * invDet;

                dpdv =
                    (e2 * duv1.x() -
                    e1 * duv2.x()) * invDet;
            }

            Vector3 dndu(0.0);
            Vector3 dndv(0.0);

            if (m_shadingNormals && std::abs(det) >= EPSILON) {

                // Derivative of the unnormalized interpolated normal.
                Vector3 dn1 = n1 - n0;
                Vector3 dn2 = n2 - n0;

                real invDet = 1.0 / det;

                Vector3 dnduLinear =
                    (dn1 * duv2.y() -
                    dn2 * duv1.y()) * invDet;

                Vector3 dndvLinear =
                    (dn2 * duv1.x() -
                    dn1 * duv2.x()) * invDet;


                // Recover barycentric coordinates from the texture UV.
                real b1;
                real b2;

                if (m_uvCoordinates) {

                    Vector2 relative = uv - uv0;

                    b1 =
                        (relative.x() * duv2.y() -
                        relative.y() * duv2.x()) * invDet;

                    b2 =
                        (duv1.x() * relative.y() -
                        duv1.y() * relative.x()) * invDet;

                } else {

                    // In the barycentric fallback, uv directly stores
                    // (b1, b2).
                    b1 = uv.x();
                    b2 = uv.y();
                }

                real b0 = 1.0 - b1 - b2;

                // Unnormalized interpolated shading normal.
                Vector3 interpolatedNormal =
                    n0 * b0 +
                    n1 * b1 +
                    n2 * b2;

                real normalLength = interpolatedNormal.length();

                if (normalLength > EPSILON) {

                    // Derivative of a normalized vector:
                    //
                    // dn = (dN - n(n . dN)) / |N|
                    //
                    dndu =
                        (dnduLinear -
                        shadingNormal *
                        shadingNormal.dot(dnduLinear))
                        / normalLength;

                    dndv =
                        (dndvLinear -
                        shadingNormal *
                        shadingNormal.dot(dndvLinear))
                        / normalLength;
                }
            }

            Vector3 s = (dpdu - shadingNormal * (shadingNormal.dot(dpdu))).normalized();

            // The second derivatives of p are all 0
            return SurfaceDifferentials(dpdu, dpdv, dndu, dndv,
                Vector3(0.0), Vector3(0.0), Vector3(0.0), s);
        }


        Vector3 getPosition(const Vector2& uv, int triangleIndex) const override{

            if (!m_uvCoordinates) {
                // Fallback
                return Barycentric::interpolate(v0, v1, v2, uv);
            }

            // Inverts UV coordinates
            Vector2 duv1 = uv1 - uv0;
            Vector2 duv2 = uv2 - uv0;
            Vector2 relative = uv - uv0;

            real det = duv1.x() * duv2.y() -
                duv2.x() * duv1.y();

            assert(std::abs(det) >= EPSILON && "Degenerate UV mapping");

            real invDet = 1.0 / det;

            real b1 = (relative.x() * duv2.y() -
                relative.y() * duv2.x()) * invDet;

            real b2 = (duv1.x() * relative.y() -
                duv1.y() * relative.x()) * invDet;

            real b0 = 1.0 - b1 - b2;

            return v0 * b0 + v1 * b1 + v2 * b2;
        }


    };

}

#endif