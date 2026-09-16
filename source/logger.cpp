
#include "../include/headers/logger.hpp"

namespace pe {
    std::ostream* Logger::output = &std::cout;
    float Logger::MAX_TIME_INTERVAL = 0.2f;
    std::unordered_map<std::string, float> Logger::lastLogged;
    std::mutex pe::Logger::mutex;
}