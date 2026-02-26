
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
            return Intersection(t, point, normal, normal, nullptr);
        }

    public:

        Sphere(): Shape(Space::LOCAL) {}


        virtual real getSurfaceArea() const{
            // For a sphere with radius 1
            return 4 * PI;
        }


        virtual AxisAlignedBox getBoundingBox() const {
            // The local sphere will range between -1 and 1
            // for all axes.
            return AxisAlignedBox(Vector3(-1.0), Vector3(1.0));
        }

        
        // Intersects the shape with a ray
        virtual std::vector<Intersection> intersect(const Ray& ray) const{
            // Solving for the sphere intersection can be done
            // by plugging the parametric ray equation into the implicit
            // unit sphere equation:
            // || o+td ||^2 = 1
            // Which reduces to a quadratic of the form:
            // t^2||d||^2 + 2t<d, o> + ||o||^2 - 1 = 0
            // at^2 + bt + c = 0
            // Which can be solved by checking the discriminant.

            std::vector<Intersection> intersections;
            
            // The norm of the direction is 1
            real a = 1;
            real b = 2 * ray.direction().dot(ray.origin());
            real c = ray.origin().lengthSquared() - 1;

            real discriminant = b * b - 4 * a * c;

            if(discriminant < 0){
                return intersections;
            }
            else if (discriminant < EPSILON){
                // Only one solution, which is -b/2a
                real t = -b / (2.0 * a);
                intersections.push_back(generateIntersection(ray, t));
            }
            else{
                // In this case we have two solutions
                real sqrtDiscriminant = sqrtReal(discriminant);
                real t0 = -b - sqrtDiscriminant / (2.0 * a);
                real t1 = -b + sqrtDiscriminant / (2.0 * a);
                
                intersections.push_back(generateIntersection(ray, t0));
                intersections.push_back(generateIntersection(ray, t1));
            }

            return intersections;
        }


        virtual AreaSample sampleSurfaceArea() const{
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