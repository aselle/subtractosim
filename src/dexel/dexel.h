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
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>
#include <cassert>

constexpr bool DEBUG = false;

struct Dexel;

template <typename T, int d> struct V {
    using VV = V<T,d>;
    union{
        T _X[d];
        struct{
            T x,y,z;
        };
    };
    V() {
        for(int k = 0; k < d; k++) _X[k] = T();
    }
    explicit V(T x, T y) :x(x), y(y) {}
    explicit V(T x, T y, T z) :x(x), y(y), z(z) {}
    template<class T2> V(const V<T2,d>& o) {
        for(int k=0;k<d;k++) _X[k] = static_cast<T>(o._X[k]);
    }
    const T& operator[](int k) const{return _X[k];}
    T& operator[](int k) {return _X[k];}
    V<T,d> operator-() const {
        V<T,d> ret;
        for(int k=0;k<d;k++) ret[k] = -(*this)[k];
        return ret;
    }
    V<T,d> operator/(float s) const {
        V<T,d> ret;
        for(int k=0;k<d;k++) ret[k] = (*this)[k] / s;
        return ret;
    }
    V<T,d> operator*(float s) const {
        V<T,d> ret;
        for(int k=0;k<d;k++) ret[k] = (*this)[k] * s;
        return ret;
    }
    V<T,d> operator-(const V<T,d>& o) const {
        V<T,d> ret;
        for(int k=0;k<d;k++) ret[k] = (*this)[k] - o[k];
        return ret;
    }
    V<T,d> operator+(const V<T,d>& o) const {
        V<T,d> ret;
        for(int k=0;k<d;k++) ret[k] = (*this)[k] + o[k];
        return ret;
    }
    float dot(const V<T,d>& o) const {
        float sum = 0.f;
        for(int k=0;k<d;k++) sum += (*this)[k] * o[k];
        return sum;
    }
    V<T,d> normalized() const {
        float sum = 0.f;
        for(int k= 0;k<d;k++) sum += (*this)[k]*(*this)[k];
        return (*this) / std::sqrt(sum);
    }
};
using V2i = V<int,2>;
using V3i = V<int,3>;
using V3f = V<float,3>;
using V3d = V<double,3>;
template<typename T,int d>
std::ostream& operator<<(std::ostream& o, const V<T,d>& v) {
    for(int k=0;k <d;k++) o<<v[k] << " ";
    return o;
}

struct Ray {
    V3f o, d;

    V3f operator()(float t) const {
        return o + d*t;
    }
};

struct Box3f {
    V3f min, max;

    Box3f(const V3f& min, const V3f& max) 
    :min(min),max(max){

    }
};

struct DexelP {
    float min=std::numeric_limits<float>::max(), max=-std::numeric_limits<float>::max();
    V3f nmin, nmax;
    char cmin, cmax;

    DexelP() {}

    DexelP(float t0, float t1, const V3f& n0, const V3f& n1,char cmin, char cmax) 
    :min(t0), max(t1), nmin(n0), nmax(n1), cmin(cmin), cmax(cmax){

    }

    bool intersects(const DexelP& o) const {
        return min <= o.max && o.min <= max;
    }
    DexelP add(const DexelP& o) const {
        DexelP ret;
        
        if(min < o.min) {
            ret.min = min;
            ret.nmin = nmin;
            ret.cmin = cmin;
        } else {
            ret.min = o.min;
            ret.nmin = o.nmin;
            ret.cmin = o.cmin;
        }
        if (max > o.max) {
            ret.max = max;
            ret.nmax = nmax;
            ret.cmax = cmax;
        } else {
            ret.max = o.max;
            ret.nmax = o.nmax;
            ret.cmax = o.cmax;
        }
        return ret;
        //return DexelP{std::min(min, o.min) , std::max(max, o.max)};
    }
    bool valid() const {
        return min < max;
    }

    bool operator==(const DexelP& o) const {
        return o.min == min && o.max == max; 
    }
    bool operator!=(const DexelP& o) const {
        return !(*this == o);
    }
};

std::ostream& operator<<(std::ostream& o, const DexelP& d) {
    o<<"("<<d.min<<","<<d.max<<")";
    return o;
}

struct Dexel {
    using DList = std::vector<DexelP>;
    DList pieces;

    Dexel(DList pieces)
        :pieces(std::move(pieces))
    {}

    Dexel()
    {}

    Dexel cut(const Dexel& other) {
        // Let's do crappy quadratic algo here
        const DList& a = pieces;
        const DList& b = other.pieces; 
        if(!a.size()) return a;
        DList u;
        for(size_t i = 0; i < a.size(); i++) {
            DexelP curr = a[i];
            for(size_t j = 0; j < b.size(); j++) {
                const DexelP& cutter = b[j];

                if(!curr.intersects(cutter)) continue;
                if(cutter.min >= curr.min && cutter.max <= curr.max) {
                    // breka into two intervals, smaller one cannot be intersected again
                    u.emplace_back(curr.min, cutter.min, curr.nmin, -cutter.nmin, curr.cmin, cutter.cmin);
                    curr = {cutter.max, curr.max, -cutter.nmax, curr.nmax, cutter.cmax, curr.cmax};
                } else if(cutter.max <= curr.max) {
                    curr = {cutter.max, curr.max, -cutter.nmax, curr.nmax, cutter.cmax, curr.cmax};
                } else if(cutter.min >= curr.min) {
                    curr = {curr.min, cutter.min, curr.nmin, -cutter.nmin, curr.cmin, cutter.cmin};
                } else if(cutter.min <= curr.min && cutter.max >= curr.max) {
                    // TODO: this seems redundant chekcs are done here
                    curr = {};
                }else{
                    // no intersection so no cut.
                    std::cout<<"No interescty"<<std::endl;
                }
            }
            if(curr.valid()) u.push_back(curr);
        }
        return u;
    }

    Dexel add(const Dexel& o) {
        size_t i = 0;
        size_t j = 0;
        const DList &a = pieces, &b = o.pieces;
        // if(b.size() == 0) return ;
        // if(a.size() == 0) pieces = a;
        DList u;
        DexelP curr;
        while(1) {
            const DexelP* next = nullptr;
            if(DEBUG) std::cout<<"curr "<<curr<<" i,j "<<i<<" "<<j<<std::endl;
            if(i < a.size() && j < b.size()) {
                if(a[i].min <= b[i].min) {
                    next = &a[i];
                    i++;
                }
                else {
                    next = &b[j];
                    j++;
                }
            } else if(i < a.size()) {
                next = &a[i];
                i++;
            } else if(j < b.size()) {
                next = &b[j];
                j++;
            }
            
            if(next) {
                if(curr.intersects(*next)) {
                    if(DEBUG) std::cout<<"  intersection "<<*next<<std::endl;
                    curr = curr.add(*next);
                } else {
                    if(DEBUG) std::cout<<"  no intersection "<<*next<<std::endl;
                    if(curr.valid())
                        u.emplace_back(curr); // TODO: std::move
                    curr = *next;
                }
            } else {
                if(DEBUG) std::cout<<" no next"<<std::endl;
                if(curr.valid())
                    u.emplace_back(curr);
                break;
            }
 
        }
        return u;

    }

    bool operator==(const Dexel& o) const {
        if(o.pieces.size() != pieces.size()) return false;
        for(size_t i  = 0; i < pieces.size(); i++) {
            if(pieces[i] != o.pieces[i]) return false;
        }
        return true;
    }

    bool operator!=(const Dexel& o) const {
        return !(*this == o);
    }
        
    
};
std::ostream& operator<<(std::ostream &o, const Dexel& d) {
    for(const auto& p: d.pieces) {
        o << p << " ";
    }
    return o;
}

enum class Plane {
    XY, XZ, YZ
};

class Traceable {
    public:
    virtual bool intersect(const Ray& r, Dexel& dexel) = 0;
};


struct Point {
    using scalar_type = float;
    V<scalar_type,3> P;
    V<scalar_type,3> N;
    scalar_type c;
};

class PointCloud {
public:
    //PointCloud() = default;
    //PointCloud (const PointCloud&) = delete;
    //PointCloud& operator= (const PointCloud&) = delete;
    std::shared_ptr<std::vector<Point>> points = std::make_unique<std::vector<Point>>();
};

struct
 DexelImage {
    V2i imin, imax;
    int stride;
    std::vector<Dexel> image;
    Plane plane;
    float dx;
    float one_over_dx;

    DexelImage(V2i imin, V2i imax, Plane plane, float dx)
    :imin(imin), imax(imax), plane(plane), dx(dx), one_over_dx(1.f/dx)
    {
        V2i index_count = imax-imin; //  + V2i(1,1);
        stride = index_count.x;
        image.resize(index_count.x * index_count.y);
    }

    void inplace_cut(const DexelImage& o) {
        V2i low (std::max(imin.x, o.imin.x), std::max(imin.y, o.imin.y));
        V2i high(std::min(imax.x, o.imax.x), std::min(imax.y, o.imax.y));
        // std::cout<<"intersect is "<<low<<" "<<high<<std::endl;
        for(int x = low.x; x < high.x; x++) {
            for(int y = low.y; y < high.y; y++) {
                (*this)(x,y) = (*this)(x,y).cut(o(x,y));
                //std::cout<<x<<" "<<y<<" -- "<<(*this)(x,y)<<std::endl;
            }
        }
    }

    DexelImage add(const DexelImage& o) {
        V2i low (std::min(imin.x, o.imin.x), std::min(imin.y, o.imin.y));
        V2i high(std::max(imax.x, o.imax.x), std::max(imax.y, o.imax.y));
        DexelImage ret(low, high, plane, dx);
        for(int x = low.x; x < high.x; x++) {
            for(int y = low.y; y < high.y; y++) {
                bool left_in = x >= imin.x && x < imax.x && y >= imin.y && y < imax.y;
                bool right_in = x >= o.imin.x && x < o.imax.x && y >= o.imin.y && y < o.imax.y;
                if(left_in && right_in) ret(x,y) = (*this)(x,y).add(o(x,y));
                else if(left_in) ret(x,y) = (*this)(x,y);
                else if(right_in) ret(x,y) = o(x,y);
            }
        }
        return ret;
    }

    void inplace_union(const DexelImage& o) {
        DexelImage new_image = add(o);
        std::swap(image, new_image.image);
        std::swap(imin, new_image.imin);
        std::swap(imax, new_image.imax);
        std::swap(stride, new_image.stride);
    }

    const Dexel& operator()(int x, int y) const {
        return image[stride * (y-imin.y) + (x-imin.x)];
    }

    Dexel& operator()(int x, int y) {
        return image[stride * (y-imin.y) + (x-imin.x)];
    }

    Ray world_space_ray(int x, int y) {
        if(plane == Plane::XY) {
            V3f o(x * dx, y * dx, 0.f);
            V3f d(0,0,1.f);
            return Ray({o,d});
        } else if (plane == Plane::XZ) {
            V3f o(y * dx, 0.f, x * dx);
            V3f d(0,1.f,0.f);
            return Ray({o,d});
        } else if (plane ==Plane::YZ){
            V3f o(0.f, x * dx, y * dx);
            V3f d(1.f,0.f,0.f);
            return Ray({o,d});
        }
        assert(false);
    }

    void trace(Traceable* tracer) {
        for(int x = imin.x; x < imax.x; x++)  {
            for(int y = imin.y; y < imax.y; y++)  {
                Ray r = world_space_ray(x,y);
                Dexel& dexel = (*this)(x,y);
                tracer->intersect(r, dexel);
                for(auto& d: dexel.pieces) {
                    d.min /= dx;
                    d.max /= dx;
                }
            }
        }
    }

    void save_obj(std::ostream& oo) {
        for(int x = imin.x; x < imax.x; x++)  {
            for(int y = imin.y; y < imax.y; y++)  {
                for(const auto& d: (*this)(x,y).pieces) {
                    Ray base_ray = world_space_ray(x,y);
                    V3f v1 = base_ray.o + base_ray.d * d.min * dx;
                    V3f v2 = base_ray.o + base_ray.d * d.max * dx;
                    oo<<"v "<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
                    oo<<"v "<<v2.x<<" "<<v2.y<<" "<<v2.z<<"\n";
                }
            }
        }
    }

    void get_pointcloud(PointCloud& cloud) {
        auto& points = *cloud.points;
        for(int x = imin.x; x < imax.x; x++)  {
            for(int y = imin.y; y < imax.y; y++)  {
                for(const auto& d: (*this)(x,y).pieces) {
                    Ray base_ray = world_space_ray(x,y);
                    V3f v1(base_ray(d.min*dx));
                    V3f v2(base_ray(d.max*dx));
                    V3f n1(d.nmin);
                    V3f n2(d.nmax);
                    float c1 = d.cmin;
                    float c2 = d.cmax;
                    points.emplace_back(Point{v1, n1, c1});
                    points.emplace_back(Point{v2, n2, c2});
                }
            }
        }
    }
};



struct TriDexelImage {
    float dx;
    Box3f box;
    std::unique_ptr<DexelImage> xy, xz, yz;

    TriDexelImage(float dx, const Box3f& box)
    :dx(dx),box(box)
    {
        V3f bmin = box.min / dx;
        V3f bmax = box.max / dx;
        V3i idxmin(static_cast<int>(bmin.x - 10.f) , static_cast<int>(bmin.y - 10.f),static_cast<int>(bmin.z -10.f)) ;
        V3i idxmax(static_cast<int>(bmax.x + 10.f), static_cast<int>(bmax.y + 10.f), static_cast<int>(bmax.z + 10.f)) ;
        xy = std::make_unique<DexelImage>(V2i(idxmin.x, idxmin.y), V2i(idxmax.x, idxmax.y), Plane::XY, dx);
        xz = std::make_unique<DexelImage>(V2i(idxmin.z, idxmin.x), V2i(idxmax.z, idxmax.x), Plane::XZ, dx);
        yz = std::make_unique<DexelImage>(V2i(idxmin.y, idxmin.z), V2i(idxmax.y, idxmax.z), Plane::YZ, dx);
    }

    void inplace_cut(const TriDexelImage& o) {
        xy->inplace_cut(*o.xy);
        xz->inplace_cut(*o.xz);
        yz->inplace_cut(*o.yz);
    }

    void inplace_union(const TriDexelImage& o) {
        xy->inplace_union(*o.xy);
        xz->inplace_union(*o.xz);
        yz->inplace_union(*o.yz);
    }

    void trace(Traceable* tracer) {
        xy->trace(tracer);
        xz->trace(tracer);
        yz->trace(tracer);
    }


    void save_obj(const char* filename)     {
        std::ofstream of(filename);
        // of << "# TEST\n";
        xy->save_obj(of);
        xz->save_obj(of);
        yz->save_obj(of);
    }

    PointCloud get_pointcloud() {
        PointCloud pc;
        xy->get_pointcloud(pc);
        xz->get_pointcloud(pc);
        yz->get_pointcloud(pc);
        return pc;
    }
};



struct Sphere : public Traceable{
    V3f center;
    float radius;
    char color;

    Sphere(const V3f& center, float radius, char color=0) : center(center), radius(radius), color(color) {}
    Sphere& operator=(const Sphere&) = default;
    Sphere(const Sphere&) = default;

    bool intersect(const Ray& r, Dexel& dexel){
        // A + tB
        float tmin, tmax;
        float a = r.d.dot(r.d);
        V3f o_minus_c = r.o - center;
        float b = 2.f*r.d.dot(o_minus_c);
        float c = o_minus_c.dot(o_minus_c) - radius*radius;
        float disc = b*b - 4.f*a*c;
        float a2 = (2*a);
        if(disc > 0) {
            tmin = (-b + std::sqrt(disc))/a2;
            tmax = (-b - std::sqrt(disc))/a2;
            if(tmin > tmax) std::swap(tmin,tmax); 
            // TODO: fix sphere
            V3f nmin = r(tmin).normalized();
            V3f nmax = r(tmax).normalized();

            dexel.pieces.emplace_back(DexelP{tmin,tmax, nmin, nmax, color, color});
            return true;
        } else {
            return false;
        }
    }

    Box3f box() {
        V3f R(radius,radius,radius);
        return Box3f{center - R, center + R};
    }
};

struct Box: public Traceable {
    V3f low;
    V3f high;
    char color;

    Box(V3f low, V3f high, char color=0)
    :low(low), high(high), color(color)
    {}

    Box3f box() const {
        return Box3f(low, high);
    }

    bool intersect(const Ray& r, Dexel& dexel) {
        float epsilon = 1e-5f;
        float tmin, tmax;
        float t[6];
        int index[] = {0,1,2,3,4,5};
        t[0] = -(r.o.x - low.x) / r.d.x;
        t[1] = -(r.o.x - high.x) / r.d.x;
        t[2] = -(r.o.y - low.y) / r.d.y;
        t[3] = -(r.o.y - high.y) / r.d.y;
        t[4] = -(r.o.z - low.z) / r.d.z;
        t[5] = -(r.o.z - high.z) / r.d.z;
        std::sort(index, index+6, [&t](int i, int j){return t[i] < t[j];});


        bool valid[6];
        for(int i=0;i<6;i++) {
            float tval = t[index[i]];
            valid[i] = !isnan(tval);
            if(!valid) continue;
            V3f p = r(tval);
            const int axis = i >> 1;
            for(int k = 0; k<3; k++) {
                if(
                    (p[k] < low[k]-epsilon || p[k] > high[k]+epsilon)) {
                        valid[i] = false;
                        break;
                }
            }
        }

        auto normals = [](int si)->V3f {
            int axis = si>>1;
            V3f n(0,0,0);
            n[axis] = (si & 1) ? 1.f : -1.f;
            return n;
        };

        bool succ = false;
        V3f nmin, nmax;
        for(int ii=0;ii<6;ii++) {
            int si = index[ii];
            if(valid[ii]) {
                tmin  = t[si];
                succ = true;
                nmin = normals(si);
                break;
            }
        }
        for(int ii=5;ii>=0;ii--) {
            int si = index[ii];
            if(valid[ii]) {
                tmax  = t[si];
                nmax = normals(si);
                break;
            }
        }
        if(succ) {
            dexel.pieces.emplace_back(DexelP{tmin, tmax, nmin, nmax, color, color});
            // std::cout<<"tmin tmax "<<tmin<< " "<< tmax<<std::endl;
        }
        return succ;
    }
};

struct Cylinder: public Traceable {
    V3f center;
    float height;
    float radius;
    char color;

    Cylinder(V3f center, float height, float radius, char color)
    :center(center), height(height), radius(radius), color(color)
    {}

    bool intersect(const Ray& r, Dexel& dexel) {
        float tmin, tmax;
        V3f o = r.o - center;
        V3f n(0,1,0);
        float dir_n = r.d.dot(n);
        V3f dir_t = r.d - n * dir_n;

        tmin = -o.y / dir_n;
        tmax = -(o.y - height) / dir_n;
        if(tmin > tmax) std::swap(tmin, tmax);


        // std::cout<<"got "<<tmin<< " "<<tmax<<std::endl;

        V3f pmin = dir_t * tmin + o;
        pmin.y = 0;
        bool min_valid = !isnan(tmin) && (pmin.dot(pmin) <= radius*radius);
        V3f pmax = dir_t * tmax + o;
        pmax.y = 0;
        bool max_valid = !isnan(tmax) && (pmax.dot(pmax) <= radius*radius);
        V3f nmin (0.f,-1.f,0.f);
        V3f nmax (0.f,1.f,0.f);
        // return  min_valid && max_valid;

        V3f o2(o.x,0,o.z);
        V3f d2(r.d.x,0, r.d.z);
        float c = o2.dot(o2) - radius*radius;
        float b = 2.f*d2.dot(o2);
        float a = d2.dot(d2);
        float disc = b*b - 4.f*a*c;
        float a2 = (2*a);
        if(disc > 0) {
            float tmin2 = (-b + std::sqrt(disc))/a2;
            float tmax2 = (-b - std::sqrt(disc))/a2;
            if(tmin2 > tmax2) {
                std::swap(tmin2,tmax2);
            }
            float tminy = o.y + tmin2 * dir_n;
            float tmaxy = o.y + tmax2 * dir_n;
            if(!min_valid && tminy >= 0 && tminy < height) {
                tmin = tmin2;
                V3f cvec = r(tmin2) - center;
                cvec[1] = 0.f;
                nmin = (cvec).normalized();
            }
            if(!max_valid && tmaxy >= 0 && tmaxy < height) {
                tmax = tmax2;
                V3f cvec = r(tmax2) - center;
                cvec[1] = 0.f;
                nmax = (cvec).normalized();
            }

            dexel.pieces.emplace_back(DexelP{tmin,tmax, nmin, nmax, color, color});
            return true;
        } else {
            if(min_valid && max_valid) {
                dexel.pieces.emplace_back(DexelP{tmin,tmax, nmin, nmax, color, color});
                return true;
            }
        }
        return false;
    }

    Box3f box() {
        V3f R(radius,0,radius);
        return Box3f{center - R, center + V3f(0,height,0) + R};
    }
};
