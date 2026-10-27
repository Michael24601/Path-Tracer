
#include "logger.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <cstring>

namespace pathtracer {

    // Default value is the console
    std::ostream* Logger::output = &std::cout;

    // Be default, not max time interval
    float Logger::MAX_TIME_INTERVAL = -1.0f;

    bool Logger::OUTPUT_TO_CONSOLE = true;

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

        std::ostringstream prefix_stream;
        prefix_stream << "["
                    << std::setw(10)
                    << std::setfill('0')
                    << std::fixed
                    << std::setprecision(3)
                    << time
                    << "s] ";

        const char* tag   = "";
        const char* color = "";

        switch(level){
            case Level::Info:    
                tag = "[INFO]";    
                color = "\033[38;2;152;196;124m";
                break;
            case Level::Warning: 
                tag = "[WARNING]"; 
                color = "\033[38;2;225;193;127m";
                break;
            case Level::Error:   
                tag = "[ERROR]";  
                color = "\033[38;2;205;49;49m";
                break;
            case Level::Trace:   
                tag = "[TRACE]";   
                color = "\033[38;2;36;114;200m";
                break;
        }

        std::ostringstream location_stream;
        location_stream << file << ":" << line << " (" << function << ")";

        std::ostringstream rest_stream;
        rest_stream << " "
                    << std::left << std::setfill(' ') << std::setw(50)
                    << location_stream.str()
                    << " " << message << "\n";

        const std::string prefix = prefix_stream.str();
        const std::string rest   = rest_stream.str();

        std::ostringstream tag_stream;
        tag_stream << std::left << std::setfill(' ') << std::setw(12) << tag;

        const std::string text    = prefix + tag_stream.str() + rest;
        const std::string colored = prefix + color + tag + "\033[0m"
                                  + std::string(12 - std::strlen(tag), ' ')
                                  + rest;
   
        // Log file (could be console)
        // Note that ina  file, we can't write colors.
        if(output == &std::cout){
            *output << colored;
            output->flush();
        } else {
            *output << text;
            output->flush();
        }


        // We also print it in console regardless
        if(OUTPUT_TO_CONSOLE && output != &std::cout){
            std::cout << colored;
            std::cout.flush();
        }
    }


    void Logger::setOutput(std::ostream& stream) {
        std::lock_guard<std::mutex> lock(mutex);
        output = &stream;
    }


    void Logger::setConsoleOutput(bool consoleOutput){
        OUTPUT_TO_CONSOLE = consoleOutput;
    }

    
    void Logger::setMaxTimeInterval(float seconds) {
        std::lock_guard<std::mutex> lock(mutex);
        MAX_TIME_INTERVAL = seconds;
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


    void Logger::trace(const std::string& message, const char* file,
        int line, const char* function) {
        log(Level::Trace, message, file, line, function);
    }

}