
#include "../include/headers/logger.hpp"

namespace pathtracer {
    std::ostream* Logger::output = &std::cout;
    float Logger::MAX_TIME_INTERVAL = -1.0f;
    std::unordered_map<std::string, float> Logger::lastLogged;
    std::mutex Logger::mutex;
}