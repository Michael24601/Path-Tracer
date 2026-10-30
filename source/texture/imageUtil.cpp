
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


        inline void linearInterpolate(const real* a, const real* b, real t,
            real* out, int channels) {

            for (int c = 0; c < channels; ++c) {
                out[c] = a[c] + (b[c] - a[c]) * t;
            }
        }


        void nearest(const Vector2& decimal, const Vector2i& floor,
            const Vector2i& ceiling, const real* data, int width, int channels,
            real* out) {

            int x = (decimal.x() < 0.5f ? floor.x() : ceiling.x());
            int y = (decimal.y() < 0.5f ? floor.y() : ceiling.y());

            // Here we just use raw arrays because we want to avoid copies
            // and allocations in low level code
            const real* p = data + (size_t(y) * width + x) * channels;
            for (int c = 0; c < channels; c++) {
                out[c] = p[c];
            }
        }


        void bilinear(const Vector2& decimal, const Vector2i& floor,
            const Vector2i& ceiling, const real* data, int width, int channels,
            real* out) {

            real tx = decimal.x();
            real ty = decimal.y();

            // Again we use raw arrays to avoid costly allocations
            const real* p00 = data + (size_t(floor.y()) * width + floor.x()) * channels;
            const real* p10 = data + (size_t(floor.y()) * width + ceiling.x()) * channels;
            const real* p01 = data + (size_t(ceiling.y()) * width + floor.x()) * channels;
            const real* p11 = data + (size_t(ceiling.y()) * width + ceiling.x()) * channels;

            // These are the two horizontal and vertical linear interpolations
            for (int c = 0; c < channels; ++c) {
                const real t0 = p00[c] + (p10[c] - p00[c]) * tx;
                const real t1 = p01[c] + (p11[c] - p01[c]) * tx;
                out[c] = t0 + (t1 - t0) * ty;
            }
        }

    }

}