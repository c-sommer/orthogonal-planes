/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2017, Christiane Sommer.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef TIMER_H_
#define TIMER_H_

// includes
#include <iostream>
#include <ctime>
#include <string>

class Timer {

private:
    clock_t start_time;
    clock_t end_time;
    double elapsed;

public:
    Timer() : start_time(0.), end_time(0.), elapsed(0.) {}
    ~Timer() {}
    
    void tic() {
        start_time = clock();
    }
    
    double toc(std::string s = "Time elapsed") {
        if (start_time!=0) {
            end_time = clock();
            elapsed = double(end_time-start_time) / CLOCKS_PER_SEC;
            print_time(s);
        }
        else
            std::cout << "Timer was not started, no time could be measured." << std::endl;
        return elapsed;
    }
    
    void print_time(std::string s = "Time elapsed") {        
        if (elapsed<1.)
            std::cout << "---------- " << s << ": " << 1000.*elapsed << "ms." << std::endl;
        else
            std::cout << "---------- " << s << ": " << elapsed << "s." << std::endl;
    }

};

#endif // TIMER_H_
