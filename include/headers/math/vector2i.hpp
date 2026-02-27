
#ifndef PATH_TRACER_VECTOR2I_HPP
#define PATH_TRACER_VECTOR2I_HPP

#include "../config.hpp"
#include "constants.hpp"

namespace pathtracer{

    class Vector2i{

    private:

        int m_data[2];
        
    public: 
        
        Vector2i(){ m_data[0] = m_data[1] = 0; }


        Vector2i(int x, int y){
            m_data[0] = x;
            m_data[1] = y;
        }


        Vector2i(int x){
            m_data[0] = m_data[1] = x;
        }


        int x() const{ return m_data[0]; }


        int y() const{ return m_data[1]; }


        // Access using brackets
        const int& operator[](int index) const {
            assert((index >= 0 && index <= 1) && "Index out of bounds");
            return m_data[index];
        }


        // Setter using brackets
        int& operator[](int index) {
            assert((index >= 0 && index <= 1) && "Index out of bounds");
            return m_data[index];
        }
    };
}

#endif