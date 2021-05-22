#include <chrono>

class Stopwatch {
public:
    size_t start_count = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::duration<double> elapsed_time = std::chrono::duration<double>::zero();
    bool started = false;

    void reset() {
        elapsed_time = std::chrono::duration<double>::zero();
    }

    void start() {
        ++start_count;
        start_time = std::chrono::system_clock::now();
        started = true;
    }

    void stop() {
        if(started) {
            elapsed_time += std::chrono::system_clock::now() - start_time;
            started = false;
        }
    }

    double get_elapsed_seconds() {
        auto elapsed = started ? std::chrono::duration<double>(elapsed_time + (std::chrono::system_clock::now() - start_time)) : elapsed_time; 
        return elapsed.count();
    }
};