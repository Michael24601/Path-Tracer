#ifndef LOGGER
#define LOGGER

#include <ostream>
#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <unordered_map>
#include <mutex>


namespace pe {

class Logger {

public:

    enum class Level {
        Info,
        Warning,
        Error
    };

private:

    static std::ostream* output;
    static float MAX_TIME_INTERVAL;

    // Keeps track of logs, ensures the same message is not spammed
    static std::unordered_map<std::string, float> lastLogged;

    static std::mutex mutex;

    static double getTime(){
        using namespace std::chrono;
        static auto start = high_resolution_clock::now();
        auto now = high_resolution_clock::now();
        return duration<double>(now - start).count();
    }

    static void log(
        Level level,
        const std::string& message,
        const char* file,
        int line,
        const char* function
    ){

        std::lock_guard<std::mutex> lock(mutex);

        if (!output){
            return;
        }

        // Ensures same message not logged too many times
        double time = getTime();
        std::string key = std::string(file) + message;
        auto it = lastLogged.find(key);
        if (it != lastLogged.end())
        {
            if (time - it->second < MAX_TIME_INTERVAL){
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

        switch (level){
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
    }

public:

    static void setOutput(std::ostream& stream){
        std::lock_guard<std::mutex> lock(mutex);
        output = &stream;
    }

    static void info(
        const std::string& message,
        const char* file,
        int line,
        const char* function
    ){
        log(Level::Info, message, file, line, function);
    }

    static void warning(
        const std::string& message,
        const char* file,
        int line,
        const char* function
    ){
        log(Level::Warning, message, file, line, function);
    }

    static void error(
        const std::string& message,
        const char* file,
        int line,
        const char* function
    ){
        log(Level::Error, message, file, line, function);
    }
};

}


/*
    These are defined to avoid  having to manually send the file,
    line, and function each time info is logged. The processor
    replaces the functions later so it's as if they were sent
    from that file.
*/
#define LOG_INFO(msg) \
    pe::Logger::info(msg, __FILE__, __LINE__, __FUNCTION__)

#define LOG_WARNING(msg) \
    pe::Logger::warning(msg, __FILE__, __LINE__, __FUNCTION__)

#define LOG_ERROR(msg) \
    pe::Logger::error(msg, __FILE__, __LINE__, __FUNCTION__)

#endif