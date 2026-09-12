
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
        real m_surfaceArea;
        AxisAlignedBox m_box;

    public:

        Mesh(const std::vector<Triangle>& triangles) {
            this->triangles = triangles;
            
            // The surface area is just the triangle sum
            m_surfaceArea = 0;
            for(int i = 0; i < triangles.size(); i++){
                m_surfaceArea += triangles[i].getSurfaceArea();
            }

            // The axis aligned box
            for(int i = 0; i < triangles.size(); i++){
                AxisAlignedBox b = triangles[i].getBoundingBox();
                m_box.extend(b);
            }
        }



        real getSurfaceArea() const override{
            return m_surfaceArea;
        }


        AxisAlignedBox getBoundingBox() const override{
            return m_box;
        }


        Vector3 getCentroid() const override{
            // The centroid of the mesh is that of its box
            return (m_box.minCorner() + m_box.maxCorner()) / 2.0f;;
        }

        
        // Intersects the shape with a ray
        Intersection intersect(const Ray& ray, real oldT) const override{

            // Not a hit by default
            Intersection result;
            
            // Replace later with a bottom level acceleration structure
            for(int i = 0; i < triangles.size(); i++){
                Intersection it = triangles[i].intersect(ray, oldT);
                if(it && it.t() < oldT && it.t() < result.t()){
                    result = it;
                }
            }

            return result;
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
            int index = point.triangleIndex;
            AreaSample sample = 
                triangles[index].evaluateAreaSample(point);
            real selectionPdf = UniformTriangle::pdf(triangles, index);
            sample.setPdf(sample.pdf() * selectionPdf);
            return sample;
        }

    };

}

#endif