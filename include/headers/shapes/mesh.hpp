
#ifndef PATH_TRACER_MESH_HPP
#define PATH_TRACER_MESH_HPP

#include "triangle.hpp"
#include "meshUtil.hpp"

namespace pathtracer{

    class Mesh : public Shape {

    private:

        // A mesh is just a collection of triangles
        std::vector<Triangle> triangles;

        // Surface area and bounding box stored since expensive to compute
        real surfaceArea;
        AxisAlignedBox box;

    public:

        Mesh(const std::vector<Triangle>& triangles) {
            this->triangles = triangles;
            
            // The surface area is just the triangle sum
            surfaceArea = 0;
            for(int i = 0; i < triangles.size(); i++){
                surfaceArea += triangles[i].getSurfaceArea();
            }

            // The axis aligned box
            Vector3 min = Vector3(INFINITY, INFINITY, INFINITY);
            Vector3 max = Vector3(-INFINITY, -INFINITY, -INFINITY);
            for(int i = 0; i < triangles.size(); i++){
                AxisAlignedBox b = triangles[i].getBoundingBox();
                min = min.elementWiseMinimum(b.minCorner());
                max = max.elementWiseMaximum(b.maxCorner());
            }
            box = AxisAlignedBox(min, max);
        }



        real getSurfaceArea() const override{
            return surfaceArea;
        }


        AxisAlignedBox getBoundingBox() const override{
            return box;
        }

        
        // Intersects the shape with a ray
        void intersect(const Ray& ray, IntersectionList& list) const override{
            // Replace later with a bottom level acceleration structure
            for(int i = 0; i < triangles.size(); i++){
                IntersectionList temp;
                triangles[i].intersect(ray, temp);
                // A triangle only generates one intersection, so it has to
                // be this one.
                if(!temp.empty()){
                    Intersection it = Intersection(*(temp.top()));
                    it.setTriangleIndex(i);
                    list.push(it);
                }
            }   
        }

            
        AreaSample sampleSurfaceArea() const override{
            // In order to sample a point on the mesh, we choose
            // at random (uniformly or proportionally to area)
            // a triangle, sample it, then combine the PDFs.
            int index = UniformTriangle::sample(triangles);
            const Triangle& triangle = triangles[index];
            real pdf = UniformTriangle::pdf(triangles, index);

            AreaSample sample = triangle.sampleSurfaceArea();
            sample.setPdf(pdf * sample.pdf());
            return sample;
        }


        AreaSample evaluateAreaSample(const SurfaceSample& point) const override{
            AreaSample sample = 
                triangles[point.triangleIndex].evaluateAreaSample(point);
            real selectionPdf = UniformTriangle::pdf(triangles, point.triangleIndex);
            sample.setPdf(sample.pdf() * selectionPdf);
            return sample;
        }

    };

}

#endif