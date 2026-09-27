
#include "texture/textureUtil.hpp"
#include <algorithm>
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "math/vector2i.hpp"

namespace pathtracer{

    namespace Border{

        static int reflect(int v, int size) {
            int period = 2 * size;
            int r = v % period;
            if (r < 0) r += period;
            if (r >= size) r = period - r - 1;
            return r;
        }


        Vector2i clamp(const Vector2i& uv, int width, int height) {
            int x = std::min(std::max(uv.x(), 0), width - 1);
            int y = std::min(std::max(uv.y(), 0), height - 1);
            return Vector2i(x, y);
        }


        Vector2i repeat(const Vector2i& uv, int width, int height) {
            int mod_x = uv.x() % width;
            int mod_y = uv.y() % height;
            int x = (mod_x >= 0) ? mod_x : mod_x + width;
            int y = (mod_y >= 0) ? mod_y : mod_y + height;
            return Vector2i(x, y);
        }


        Vector2i mirror(const Vector2i& uv, int width, int height) {
            int x = reflect(uv.x(), width);
            int y = reflect(uv.y(), height);
            return Vector2i(x, y);
        }

    }


    namespace Filter{

        Vector3 nearest(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<Vector3>>& data) {

            real tx = decimal.x();
            real ty = decimal.y();
            int x = (tx < 0.5f ? floor.x() : ceiling.x());
            int y = (ty < 0.5f ? floor.y() : ceiling.y());
            return data[y][x];
        }


        Vector3 bilinear(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<Vector3>>& data) {

            real tx = decimal.x();
            real ty = decimal.y();

            Vector3 t00 = data[floor.y()][floor.x()];
            Vector3 t01 = data[ceiling.y()][floor.x()];
            Vector3 t10 = data[floor.y()][ceiling.x()];
            Vector3 t11 = data[ceiling.y()][ceiling.x()];

            // We interpolate horizontally
            Vector3 t0 = t10 * tx + t00 * (1.0f - tx);
            Vector3 t1 = t11 * tx + t01 * (1.0f - tx);
            // Then vertically 
            Vector3 t = t1 * ty + t0 * (1.0f - ty);
            return t;
        }

    }

}