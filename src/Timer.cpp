//
//  Timer.cpp
//  Cholesky
//
//  Created by PH on 29/11/15.
//  Copyright © 2015 PH. All rights reserved.
//

#include "Timer.hpp"
#include <iostream>

Timer::Timer(std::string str)
: start(clock::now()), name(str)
{
}

void
Timer::reset()
{
    start = clock::now();
}

int
Timer::seconds() const
{
    return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(clock::now() - start).count());
}

int
Timer::milliseconds() const
{
    return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count());
}

int
Timer::microseconds() const
{
    return static_cast<int>(std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - start).count());
}

int
Timer::hours() const
{
    return static_cast<int>(std::chrono::duration_cast<std::chrono::hours>(clock::now() - start).count());
}

int
Timer::minutes() const
{
    return static_cast<int>(std::chrono::duration_cast<std::chrono::minutes>(clock::now() - start).count());
}

void
Timer::printTime() const
{
    
    using namespace std::chrono;
    
    const auto diff = clock::now() - start;
    
    int us = static_cast<int>(duration_cast<std::chrono::microseconds>(diff).count());
    int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
    int s = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(diff).count());
    int m = static_cast<int>(std::chrono::duration_cast<std::chrono::minutes>(diff).count());
    int h = static_cast<int>(std::chrono::duration_cast<std::chrono::hours>(diff).count());
    
    us -= 1000 * ms;
    ms -= 1000 * s;
    s -= 60 * m;
    m -= 60 * h;
    
    std::cout << name << ": ";
    if(h) std::cout << h << "h ";
    if(m || h) std::cout << m << "m ";
    if(s || m || h) std::cout << s << "s ";
    if(s || m || h || ms) std::cout << ms << "ms ";
    
    std::cout << us << "μs" << std::endl << std::flush;
}

