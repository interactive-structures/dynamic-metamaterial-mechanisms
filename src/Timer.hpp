#ifndef Timer_hpp
#define Timer_hpp

#include <chrono>
#include <string>

struct Timer
{
    typedef std::chrono::system_clock clock;
    typedef std::chrono::time_point<clock> timepoint;
    
    timepoint start;
    std::string name;
    
    Timer(std::string str = "Timer");
    
    void reset();
    int seconds() const;
    int milliseconds() const;
    int microseconds() const;
    int minutes() const;
    int hours() const;
    void printTime() const;
};
#endif /* Timer_hpp */
