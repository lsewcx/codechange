#include <chrono>

class Timer {
    private:
    using clock_t = std::chrono::high_resolution_clock;
    using second_t =std::chrono::duration<double,std::ratio<1>>;

    std::chrono::time_point<clock_t> start_time;  


public:
    void start(){
        start_time = clock_t::now();
    }

    double elapsed(){
        std::chrono::time_point<clock_t> end_time = clock_t::now();
        return std::chrono::duration_cast<second_t>(end_time - start_time).count();
    }

    void reset()
    {
        start_time = clock_t::now();
    }
};