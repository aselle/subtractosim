/*
Licensed to the Apache Software Foundation (ASF) under one
or more contributor license agreements.  See the NOTICE file
distributed with this work for additional information
regarding copyright ownership.  The ASF licenses this file
to you under the Apache License, Version 2.0 (the
"License"); you may not use this file except in compliance
with the License.  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing,
software distributed under the License is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied.  See the License for the
specific language governing permissions and limitations
under the License.    
*/
#include "dexel.h"


void EXPECT_EQ(const Dexel& a, const Dexel& b) {
    if(a != b) {
        std::cerr<<"mismatch "<<a<< " vs " << b<<std::endl;
    }
}

void testcsg() {
    Dexel d1({{-5.f,5.f}});
    Dexel d2({{10.f,15.f}});
    Dexel d3 = d1.add(d2);
    EXPECT_EQ(d3, Dexel({{-5.f,5.f},{10.f,15.f}}));

    Dexel d4({{11.f,20.f}});
    Dexel d5 = d3.add(d4);
    EXPECT_EQ(d5, Dexel({{-5.f,5.f},{10.f,20.f}}));

    Dexel d6({{9.f,18.f}});
    Dexel d7 = d5.add(d6);
    EXPECT_EQ(d7, Dexel({{-5.f,5.f},{9.f,20.f}}));

    Dexel d8({{7.f,8.f}});
    Dexel d9 = d7.add(d8);
    EXPECT_EQ(d9, Dexel({{-5.f,5.f},{7.f, 8.f}, {9.f,20.f}}));

    Dexel d10({{-100.f,100.f}});
    Dexel d11 = d9.add(d10);
    EXPECT_EQ(d11, Dexel({{-100.f,100.f}}));


    Dexel d12({{-50.f,50.f}});
    Dexel d13 = d11.cut(d12);
    EXPECT_EQ(d13, Dexel({{-100.f,-50.f},{50.f,100.f}}));

    Dexel d14({{-120, -90}, {30, 60}});
    Dexel d15 = d13.cut(d14);
    EXPECT_EQ(d15, Dexel({{-90.f,-50.f},{60.f,100.f}}));
    
    Dexel d16({{-200,-195}});
    Dexel d17 = d15.cut(d16);
    EXPECT_EQ(d17, Dexel({{-90.f,-50.f},{60.f,100.f}}));

    Dexel d18({{-200,200}});
    Dexel d19 = d17.cut(d18);
    EXPECT_EQ(d19, Dexel()); //

}

void testimage() {
    {
        V2i cut_idx_0(-5,-5),cut_idx1(5,5);
        DexelImage cut(cut_idx_0, cut_idx1, Plane::XY, 1.f);
        for(int x = cut_idx_0.x; x < cut_idx1.x; x++) { 
            for(int y = cut_idx_0.y; y < cut_idx1.y; y++) {
                if(x*x+y*y < 16)
                    cut(x,y) = Dexel({{5.,10.}});
            }
        }
        V2i idx0(-32,-32), idx1(32,32);
        DexelImage slab(idx0, idx1, Plane::XY, 1.f);
        for(int x = idx0.x; x < idx1.x; x++) { 
            for(int y = idx0.y; y < idx1.y; y++) {
                slab(x,y) = Dexel({{-10.,10.}});
            }
        }
        slab.inplace_cut(cut);
    }

    
}

void testfull() {
    const float dx = .1f;
    Sphere sphere{V3f(-5.f,0.f,0.f), 10.f};
    Sphere sphere2{V3f(+10.f,0.f,0.f), 10.f};
    TriDexelImage tri1(dx, sphere.box());
    TriDexelImage tri2(dx, sphere2.box());

    tri1.trace(sphere);
    tri2.trace(sphere2);

    tri1.inplace_cut(tri2);
    for(int t = 0; t < 20; t++) {
        Cylinder cyl{V3f(.5f*t-5.f,0.f,.5f*t-5.f), 5.f, 1.f};
        TriDexelImage tri3(dx, cyl.box());
        tri3.trace(cyl);
        tri1.inplace_cut(tri3);
    }

    tri1.save_obj("tridex.obj");
}

int main() {
    testcsg();
    testimage();
    testfull();
    return 0;
}