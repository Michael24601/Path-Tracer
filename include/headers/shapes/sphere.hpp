
#ifndef PATH_TRACER_SPHERE_HPP
#define PATH_TRACER_SPHERE_HPP

#include "shape.hpp"
#include "../math/constants.hpp"

namespace pathtracer{

    // The sphere is in local coordinates, so we assume the
    // center is at the origin, and the radius is 1.
    class Sphere : public Shape{
    
    private:

        // Given a ray and a distance t along it, sets the intersection
        // object.
        Intersection generateIntersection(const Ray& ray, real t) const {
            Vector3 point = ray.at(t);
            // The shading and geometry normal are the same
            Vector3 normal = point.normalized();

            // UV calculation
            real theta = std::acos(Util::clamp(normal.z(), -1, 1));
            real phi   = std::atan2(normal.y(), normal.x());
            if (phi < 0){
                    phi += 2 * PI;
            }

            Vector2 uv(phi * 0.5 * INV_PI, theta * INV_PI);

            // Tangent calculation
            Vector3 tangent(-normal.y(), normal.x(), 0.0);

            if (tangent.lengthSquared() > 0){
                tangent = tangent.normalized();
            }
            else{
                tangent = Vector3(1, 0, 0);
            }
            
            return Intersection(t, point, normal, normal, tangent, 
                uv, nullptr);
        }

    public:

        Sphere() {}


        real getSurfaceArea() const override{
            // For a sphere with radius 1
            return 4 * PI;
        }


        AxisAlignedBox getBoundingBox() const override {
            // The local sphere will range between -1 and 1
            // for all axes.
            return AxisAlignedBox(Vector3(-1.0), Vector3(1.0));
        }

        
        // Intersects the shape with a ray
        void intersect(const Ray& ray, IntersectionList& list) const override{
            // Solving for the sphere intersection can be done
            // by plugging the parametric ray equation into the implicit
            // unit sphere equation:
            // || o+td ||^2 = 1
            // Which reduces to a quadratic of the form:
            // t^2||d||^2 + 2t<d, o> + ||o||^2 - 1 = 0
            // at^2 + bt + c = 0
            // Which can be solved by checking the discriminant.
            
            // The norm of the direction is 1
            real a = 1;
            real b = 2 * ray.direction().dot(ray.origin());
            real c = ray.origin().lengthSquared() - 1;

            real discriminant = b * b - 4 * a * c;

            if(discriminant < 0){
                return;
            }
            else if (discriminant < EPSILON){
                // Only one solution, which is -b/2a
                real t = -b / (2.0 * a);
                list.push(generateIntersection(ray, t));
            }
            else{
                // In this case we have two solutions
                real sqrtDiscriminant = sqrtReal(discriminant);
                real t0 = -b - sqrtDiscriminant / (2.0 * a);
                real t1 = -b + sqrtDiscriminant / (2.0 * a);
                
                list.push(generateIntersection(ray, t0));
                list.push(generateIntersection(ray, t1));
            }
        }


        AreaSample sampleSurfaceArea() const override{
            // We will sample the sphere surface area by mapping
            // a 2D square onto the sphere using spherical coordinates.
            Vector2 uv = Random::next2D();
            Vector3 point = SquareToSphereUniform::transform(uv);
            real pdf = SquareToSphereUniform::pdf(point);

            // Because the mapping is uniform, the PDF is just
            // the reciprocal of the surface area. If not, we would
            // have had to calculate the proabbility of choosing
            // this specific point and returned that value.
            AreaSample sample(point, pdf);
        }

    };
}

#endif