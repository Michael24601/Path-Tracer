
#include "renderer/renderer.hpp"
#include "integrator/integrator.hpp"
#include "math/vector3.hpp"

namespace pathtracer{


    Renderer::~Renderer(){
        if(m_integrator){
            delete m_integrator;
        }
    }


    // Renders the scene by calling render in integrator
    std::vector<std::vector<Vector3>> Renderer::render(){
        return m_integrator->render(*m_scene, *m_camera, m_width,
            m_height, reporter);
    }


    std::string Renderer::toString() const{
        return m_integrator->toString() + " "
            + std::to_string(m_height) + "x" 
            + std::to_string(m_width) + "px";
    }


    void Renderer::printStatistics() const{
        m_integrator->printStatistics(m_width, m_height);
    }

}