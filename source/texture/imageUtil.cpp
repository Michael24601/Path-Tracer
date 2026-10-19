
#include "texture/imageUtil.hpp"
#include <algorithm>
#include "math/vector2.hpp"
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


        std::vector<real> linearInterpolate(const std::vector<real>& a,
            const std::vector<real>& b, real t) {

            std::vector<real> result(a.size());

            for (int i = 0; i < a.size(); i++){
                result[i] = a[i] * (1.0f - t) + b[i] * t;
            }

            return result;
        }


        std::vector<real> nearest(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<std::vector<real>>>& data) {

            real tx = decimal.x();
            real ty = decimal.y();
            int x = (tx < 0.5f ? floor.x() : ceiling.x());
            int y = (ty < 0.5f ? floor.y() : ceiling.y());
            return data[y][x];
        }


        std::vector<real> bilinear(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<std::vector<real>>>& data) {

            real tx = decimal.x();
            real ty = decimal.y();

            std::vector<real> t00 = data[floor.y()][floor.x()];
            std::vector<real> t01 = data[ceiling.y()][floor.x()];
            std::vector<real> t10 = data[floor.y()][ceiling.x()];
            std::vector<real> t11 = data[ceiling.y()][ceiling.x()];

            // We interpolate horizontally
            std::vector<real> t0 = linearInterpolate(t00, t10, tx);
            std::vector<real> t1 = linearInterpolate(t01, t11, tx);
            // Then vertically 
            std::vector<real> t = linearInterpolate(t0, t1, ty);

            return t;
        }

    }

}