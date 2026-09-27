
#ifndef PATH_TRACER_VECTOR2I_HPP
#define PATH_TRACER_VECTOR2I_HPP

namespace pathtracer{

    class Vector2i{

    private:

        int m_data[2];

    public:

        Vector2i();

        Vector2i(int x, int y);

        Vector2i(int x);

        int x() const;

        int y() const;

        // Access using brackets
        const int& operator[](int index) const;

        // Setter using brackets
        int& operator[](int index);

    };

}

#endif