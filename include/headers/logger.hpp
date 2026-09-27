
#ifndef PATH_TRACER_LOGGER_H
#define PATH_TRACER_LOGGER_H

#include <mutex>
#include <ostream>
#include <string>
#include <unordered_map>

namespace pathtracer {

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

        static double getTime();

        static void log(Level level, const std::string& message, 
            const char* file, int line, const char* function);

    public:

        static void setOutput(std::ostream& stream);

        static void info(const std::string& message, const char* file, 
            int line, const char* function);

        static void warning(const std::string& message, const char* file, 
            int line, const char* function);

        static void error(const std::string& message, const char* file, 
            int line, const char* function);
    };

}


/*
    These are defined to avoid  having to manually send the file,
    line, and function each time info is logged. The processor
    replaces the functions later so it's as if they were sent
    from that file.
*/
#define LOG_INFO(msg) \
    pathtracer::Logger::info(msg, __FILE__, __LINE__, __FUNCTION__)

#define LOG_WARNING(msg) \
    pathtracer::Logger::warning(msg, __FILE__, __LINE__, __FUNCTION__)

#define LOG_ERROR(msg) \
    pathtracer::Logger::error(msg, __FILE__, __LINE__, __FUNCTION__)

#endif