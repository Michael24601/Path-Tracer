

#ifndef PATH_TRACER_INTERSECTION_LIST_HPP
#define PATH_TRACER_INTERSECTION_LIST_HPP

#include "../math/vector3.hpp"
#include "../shapes/shape.hpp"
#include "../core/instance.hpp"
#include "intersection.hpp"

namespace pathtracer{

    // Stores intersections, ordered according to the distance t.
    // We assume all of the intersections are in the same space
    // (local or global) as the comparison is not meaningful
    // otherwise.
    class IntersectionList{

    private:

        // This is shorthand for comparing two intersections,
        // assuming they live in the same space. It can be used
        // to sort intersections.
        struct CompareT {
            bool operator()(const Intersection& a, 
                const Intersection& b) const {

                return a.t() > b.t();
            }
        };
        
        std::priority_queue<Intersection, 
            std::vector<Intersection>, CompareT> queue;

    public:

        void push(const Intersection& hit) {
            queue.push(hit);
        }


        // Returns the nearest hit without removing it
        const Intersection* top() const {
            if (queue.empty()) {
                return nullptr;
            }
            return &queue.top();
        }


        // Removes the nearest hit
        void pop() {
            queue.pop();
        }


        // Checks if the queue is empty
        bool empty() const {
            return queue.empty();
        }


        // Clears the queue
        void clear() {
            while (!queue.empty()) {
                queue.pop();
            }
        }

    };

}

#endif