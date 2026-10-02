
#include "logger.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

namespace pathtracer {

    // Default value is the console
    std::ostream* Logger::output = &std::cout;

    float Logger::MAX_TIME_INTERVAL = -1.0f;

    std::unordered_map<std::string, float> Logger::lastLogged;

    std::mutex Logger::mutex;


    double Logger::getTime() {
        using namespace std::chrono;

        static auto start = high_resolution_clock::now();
        auto now = high_resolution_clock::now();

        return duration<double>(now - start).count();
    }


    void Logger::log(Level level, const std::string& message, 
        const char* file, int line, const char* function) {

        std::lock_guard<std::mutex> lock(mutex);

        if(!output){
            return;
        }

        // Ensures same message not logged too many times
        double time = getTime();
        std::string key = std::string(file) + message;
        auto it = lastLogged.find(key);

        if(it != lastLogged.end()){
            if(time - it->second < MAX_TIME_INTERVAL){
                return;
            }
        }

        lastLogged[key] = time;

        *output << "["
            << std::setw(10)
            << std::setfill('0')
            << std::fixed
            << std::setprecision(3)
            << time
            << "s] ";

        switch(level){
            case Level::Info:
                *output << "[INFO] ";
                break;

            case Level::Warning:
                *output << "[WARNING] ";
                break;

            case Level::Error:
                *output << "[ERROR] ";
                break;
        }

        *output << file << ":" << line
                << " (" << function << ") "
                << message << "\n";

        // Important in case the program exits early
        output->flush();
    }


    void Logger::setOutput(std::ostream& stream) {
        std::lock_guard<std::mutex> lock(mutex);
        output = &stream;
    }


    void Logger::info(const std::string& message, const char* file, 
        int line, const char* function) {
        log(Level::Info, message, file, line, function);
    }


    void Logger::warning(const std::string& message, const char* file, 
        int line, const char* function) {
        log(Level::Warning, message, file, line, function);
    }


    void Logger::error(const std::string& message, const char* file, 
        int line, const char* function) {
        log(Level::Error, message, file, line, function);
    }

}