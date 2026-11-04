
#ifndef PATH_TRACER_PROGRESS_REPORTER_HPP
#define PATH_TRACER_PROGRESS_REPORTER_HPP

#include <atomic>
#include <string>

namespace pathtracer {


    // Prints a progress report. The amount of messages, the total
    // steps, the step size... are defined by the user.
    class ProgressReporter {

    private:

        // Total number of messages
        int m_messageCount;

        // The name of the steps
        std::string m_label;

        // The total number of operations
        long long m_total;

        // The interval for operations before printing a new step
        long long m_interval;

        // Counts the number of finished operations
        std::atomic<long long> m_count;

    public:

        ProgressReporter(int messageCount);

        // Begins a new report. Start is excpected not to be called
        // on multiple threads.
        void start(long long total, const std::string& label);


        // This is called everytime an operation is complete,
        // and will also print every interval. Is thread safe.
        void step();

    };

}

#endif