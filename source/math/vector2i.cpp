
#include "math/vector2i.hpp"
#include <cassert>


namespace pathtracer{

    Vector2i::Vector2i() {
        m_data[0] = m_data[1] = 0;
    }

    Vector2i::Vector2i(int x, int y) {
        m_data[0] = x;
        m_data[1] = y;
    }

    Vector2i::Vector2i(int x) {
        m_data[0] = m_data[1] = x;
    }

    int Vector2i::x() const {
        return m_data[0];
    }

    int Vector2i::y() const {
        return m_data[1];
    }

    // Access using brackets
    const int& Vector2i::operator[](int index) const {

        assert((index >= 0 && index <= 1) && "Index out of bounds");

        return m_data[index];
    }

    // Setter using brackets
    int& Vector2i::operator[](int index) {

        assert((index >= 0 && index <= 1) && "Index out of bounds");

        return m_data[index];
    }

}