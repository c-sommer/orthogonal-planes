/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Yumin Sun.
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


#ifndef PPFPLANE_UNIONFINDFORESTINDEXANDPATHHALVING_H
#define PPFPLANE_UNIONFINDFORESTINDEXANDPATHHALVING_H

#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>
#include <iostream>

struct UnionFind {

    using ValueType = uint16_t;

    std::vector<ValueType> parents;
    std::vector<ValueType> ranks;
    std::vector<ValueType> sizes;

    static constexpr ValueType InvalidIndex = std::numeric_limits<ValueType>::max();

    void INIT(ValueType n) {
        sizes.resize(n, 1);
        parents.resize(n);
        std::iota(parents.begin(), parents.end(), 0);
        ranks.resize(n, 0);
    }

    void UNION(ValueType x, ValueType y) {
        LINK(FIND_SET(x), FIND_SET(y));
    }

    ValueType FIND_SET(ValueType x) {
        while (x != parents[x] && parents[x] != InvalidIndex) {
            parents[x] = parents[parents[x]];
            x = parents[x];
        }
        return x;
    }
    
    void LINK(ValueType root1, ValueType root2) {
        if (root1 == root2) return;
        if (root1 < root2) {
            parents[root2] = root1;
            sizes[root1] += sizes[root2];
            --sizes[root2];
        }
        else {
            parents[root1] = root2;
            sizes[root2] += sizes[root1];
            --sizes[root1];
        }
    }
  
};

#endif // PPFPLANE_UNIONFINDFORESTINDEXANDPATHHALVING_H
