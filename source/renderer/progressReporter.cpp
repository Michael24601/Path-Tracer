
#include "renderer/progressReporter.hpp"
#include "logger.hpp"
#include <algorithm>

namespace pathtracer {


    ProgressReporter::ProgressReporter(int messageCount) :
        m_messageCount{messageCount}{}
    
        

    // Begins a new report
    void ProgressReporter::start(long long total, const std::string& label){
        
        m_total = total;
        m_label = label;
        
        m_interval = m_messageCount > 0
            ? std::max(1LL, m_total / m_messageCount)
            : 0;

        // Resets count
        m_count = 0;
    }


    // This is called everytime an operation is complete,
    // and will also print every interval.
    void ProgressReporter::step(){

        // Increments m_count, then copies it into a local variable
        // for thread safety.
        long long count = ++m_count;

        // We did not reach an interval yet.
        if (m_messageCount <= 0 || count % m_interval != 0) {
            return;
        }

        // This is the step we got to.
        long long step = count / m_interval;

        // This is a safety check to ensure we haven't printed more
        // steps than we are allowed.
        if(step > m_messageCount){
            return;
        }

        std::string total = std::to_string(m_messageCount);
        std::string current = std::to_string(step);
        current.insert(0, total.size() - current.size(), ' ');

        std::string prefix = m_label.empty() ? "" : m_label + " ";
        LOG_INFO(prefix + "step [" + current + "/" + total + "] completed.");
    }

}