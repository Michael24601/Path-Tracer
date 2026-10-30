
#include "shapes/mesh.hpp"
#include "bvh/blAccelerationStructure.hpp"
#include "intersection/intersection.hpp"
#include "intersection/areaSample.hpp"
#include "shapes/meshUtil.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "intersection/hit.hpp"

namespace pathtracer{

    Mesh::Mesh(const std::vector<Triangle>& triangles) {

        this->triangles = triangles;
        m_bvh = new BlAccelerationStructure(this->triangles);
        
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


    Mesh::~Mesh(){
        delete m_bvh;
    }


    real Mesh::getSurfaceArea() const {
        return m_surfaceArea;
    }

    AxisAlignedBox Mesh::getBoundingBox() const {
        return m_box;
    }

    Vector3 Mesh::getCentroid() const {
        // The centroid of the mesh is that of its box
        return (m_box.minCorner() + m_box.maxCorner()) / 2.0f;
    }


    // Intersects the shape with a ray
    bool Mesh::intersect(const Ray& ray, Hit& oldHit) const {
        return m_bvh->intersect(oldHit, triangles, ray);
    }


    Vector2 Mesh::getUv(const Hit& hit, const Ray& ray) const {
        return triangles[hit.triangleIndex].getUv(hit, ray);
    }


    AreaSample Mesh::sampleSurfaceArea() const {
        // In order to sample a point on the mesh, we choose
        // at random (uniformly or proportionally to area)
        // a triangle, sample it, then combine the PDFs.
        int index = UniformTriangle::sample(triangles);
        const Triangle& triangle = triangles[index];
        real pdf = UniformTriangle::pdf(triangles, index);

        AreaSample sample = triangle.sampleSurfaceArea();
        sample.setTriangleIndex(index);
        sample.setPdf(pdf * sample.pdf());

        return sample;
    }

    AreaSample Mesh::evaluateAreaSample(const SurfaceSample& point) const {
        int index = point.triangleIndex;

        AreaSample sample = triangles[index].evaluateAreaSample(point);

        real selectionPdf = UniformTriangle::pdf(triangles, index);

        sample.setPdf(sample.pdf() * selectionPdf);
        sample.setTriangleIndex(index);

        return sample;
    }


    SurfaceDifferentials Mesh::computeDifferentials(const Vector3& position, 
        const Vector2& uv, int triangleIndex) const {

        return triangles[triangleIndex].computeDifferentials(position, uv);
    }


    Vector3 Mesh::getPosition(const Vector2& uv, int triangleIndex) const {
        return triangles[triangleIndex].getPosition(uv, 0);
    }


    SurfacePoint Mesh::getSurfacePoint(const Vector3& point,
        const Hit& hit) const{

        return triangles[hit.triangleIndex].generateSurfacePoint(point, 
            hit.barycentric);
    }

}