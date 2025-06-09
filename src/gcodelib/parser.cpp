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
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace py = pybind11;

#ifndef M_PI
constexpr float pi = 3.141592653589793238462643383279;
#else
constexpr float pi = float(M_PI);
#endif
constexpr float MM_PER_INCH = 25.4f;
constexpr float INCH_PER_MM = 1.f / MM_PER_INCH;

using uchar = unsigned char;

struct ModalGroupSpec {
    int num; // group num
    const char* name; // name
    std::vector<int> gcodes; // which gcodes
    int my_default; // which are current
};

constexpr int NON_MODAL_GROUP = 0;
constexpr int MOTION_GROUP = 1;
constexpr int PLANE_GROUP = 2;
constexpr int DISTANCE_GROUP = 3;
constexpr int FEED_RATE_GROUP = 5;
constexpr int UNITS_GROUP = 6;

ModalGroupSpec groups[] = {
    {NON_MODAL_GROUP, "non-modal", {4,10,28,30,53,92}, 0},
    {MOTION_GROUP, "motion", {0,1,2,3,80,81,82,83,84,85,86,87,88,89 /*TODOG38.2*/}, 0},
    {PLANE_GROUP, "plane", {17,18,19}, 17},
    {DISTANCE_GROUP, "distance", {90, 91}, 90},
    {FEED_RATE_GROUP, "feed rate", {93, 94}, 94},
    {UNITS_GROUP, "units", {20,21}, 20},
    {7, "cutter radius comp", {40,42,41}, 40},
    {8, "tool length offset", {43,49}, 43},
    {10, "return mode in canned cycle", {98,99}, 98},
    {12, "work coordinate", {54,55,56,57,58,59/*59.1, 59.2, 59.3*/}, 54},
};

std::ostream& operator<<(std::ostream& o, const std::array<float,3>& X){
    o<<X[0]<<","<<X[1]<<","<<X[2];
    return o;
}

class Interface {
public:
    void rapid(std::array<float,3> X){
        std::cout<<"rapid "<<X<<std::endl;
    }
    void linear(std::array<float,3> X, float feed){
        std::cout<<"linear "<<X<<" @ "<<feed<<std::endl;
    }
};

class Interpreter
{
public:
    enum class MachineUnits {INCH, MM};
    MachineUnits machine_units;
    size_t cnt = 0;
    float vals[256];
    float unit = 1.f;
    float x = 1.f, y = 1.f, z = 1.f;
    float dx = 0.f, dy = 0.f, dz = 0.f;
    float accuracy = 0.01f; // TODO: make configurable from CNC
    bool absolute = true;
    bool inch_machine = false; // TODO: make configurable from CNC
    bool arc_absolute = false;
    bool retract_z = true;
    int gcode = -1;
    std::array<float,3> center={0,0,0};
    std::array<float,3> offset={0,0,0};
    std::array<float,3> tool_offset={0,0,0};

    std::array<bool,256> vals_set;
    std::array<int,256> gcode_to_modal_group;
    std::array<int, 32> group_modes;
    std::array<int, 32> group_touched;

    Interface* interface;

    int _line = 0;

    int block;
    std::vector<float> pts;

    float feed;


    enum PLANE
    {
        XY,
        XZ,
        YZ
    } plane;

    Interpreter()
    {
        pts.reserve(15644);
        // Initialize the registers.
        for (int k = 0; k < 256; k++)
        {
            vals[k] = 0.f;
        }
        // Initialize the modal group loolkups
        
        for(size_t k=0;k<gcode_to_modal_group.size();k++) {
            gcode_to_modal_group[k] = -1;
        }
        for(const auto& group : groups) {
            for(const int code: group.gcodes) {
                if(gcode_to_modal_group[code] != -1) 
                    throw std::runtime_error("Code " + std::to_string(code) +" used multiple times.");
                gcode_to_modal_group[code] = group.num;
                group_modes[group.num] = group.my_default;
                group_touched[group.num] = 0;
            }
        }
    }

    void reset()
    {
        pts.clear();
        cnt = 0;
    }

    void execute() {
        // g93 g94
        // set feed
        // set spindle speed
        // select tool T
        // change tool M6
        // spindle on/off m3,m4,m5
        // coolant m7,m8,m9
        // dwell g4
        // plane g17 g18 g19
        // g20/g21 length units
        // g40 g41 g42 cutter radius comp
        // cutter length comp g43/g49
        // g54 ... g59.3 work coord

        if(group_touched[12] == 1) {
            int work_offset = group_modes[12];
            std::cout<<"changed offset to G"<<group_modes[12]<<std::endl;
        }
        // set path control mode (g61 g61.1 g64)
        // g90 g91 // incremental mode
        // g98 g99 // retract mode
        // g28, g30, g10, g92, g92.1, g92.2, g94 homing
        // g0-g3 g80-g89 g53 perform motion
        
        auto get_linear_coord = [&](int axis, char c, float& final)->bool {
            if(vals_set[c]) {
                float raw = vals[c];
                if(group_modes[UNITS_GROUP] == 20 && machine_units==MachineUnits::MM) {
                    raw *= 25.4f;
                } else if(group_modes[UNITS_GROUP] == 21 && machine_units==MachineUnits::INCH){
                    raw /= 25.4f;
                }
                if(group_modes[NON_MODAL_GROUP] == 53) {
                    final = raw;
                } else {
                    if(group_modes[DISTANCE_GROUP] == 90) {
                        final = raw + offset[axis];
                    } else if(group_modes[DISTANCE_GROUP] == 91) {
                        final += raw;
                    }
                    else throw std::runtime_error("Not G90 or G91");
                }
                return true;
            } else {
                return false;
            }
        };
        

        if(group_modes[1] == 0 || group_modes[1] == 1) {
            auto end = center;
            bool has_move = false;
            for(int k=0;k<3;k++) has_move |= get_linear_coord(k, 'X'+k, end[k]);
            if(has_move) {
                if(group_modes[MOTION_GROUP] == 0) interface->rapid(end);
                else if(group_modes[MOTION_GROUP] == 1) interface->linear(end, feed);
                else throw std::runtime_error("Unexpected G-code in motion modal group.");
            }
        } 
        // stop

    }

    void handle_g(float val)
    {
        int gcode_raw = int(val);
        int decimal = int(10 * (val - gcode_raw));
        
        int group = gcode_to_modal_group[gcode_raw];
        std::cout<<"gcode raw "<<gcode_raw<<" group "<<group<<std::endl;
        if(group != -1) {
            if(group_touched[group] != 0) {
                throw std::runtime_error("multiple modal group in same line");
            }
            group_touched[group] = 1;
            std::cout<<"group mode "<<group<<" decimal "<<decimal<<std::endl;
            group_modes[group] = gcode_raw; // TODO: this doesn't work with decimnal
        } else {
            throw std::runtime_error("Invalid code G" + std::to_string(gcode_raw));
        }

        #if 0
        int gcode_raw = int(val);
        // if(gcode_raw == 2 || gcode_raw == 3) assert(false);
        int decimal = int(10 * (val - gcode_raw));
        switch (gcode_raw)
        {
        case 4:
        case 10:
        case 53:
            break;
        case 17:
            plane = XY;
            break;
        case 18:
            plane = XZ;
            break;
        case 19:
            plane = YZ;
            break;
        case 20:
            unit = inch_machine ? 1.0f : MM_PER_INCH;
            break;
        case 21:
            // TODO: fix
            unit = inch_machine ? INCH_PER_MM : 1.0f;
            break;
        case 80:
            gcode = 0;
            dz = 0;
            vals['Z'] = z;
            break;
        case 90:
            if (decimal == 0)
                absolute = true;
            else if (decimal == 1)
                arc_absolute = true;
            break;
        case 91:
            if (decimal == 0)
                absolute = false;
            else if (decimal == 1)
                arc_absolute = false;
            break;
        case 93:
        case 94:
        case 95:
            // TODO: feedmode
            break;
        case 98:
            retract_z = true;
            break;
        case 99:
            retract_z = false;
            break;
        }
        gcode = gcode_raw;
        #endif
    }

    void handleCmd(const char *cmd)
    {
        char code = toupper(cmd[0]);

        // TODO robust
        // printf("code is %c -- cmd is %s\n", code, cmd.c_str());
        float value = static_cast<float>(atof(cmd + 1));
        handleCmd(code, value);
    }

    void handle_m(float value) {
    }

    void handleCmd(uchar code, float value) {
        std::cout<<"code"<<code<<" val "<<value<<std::endl;
        switch (code)
        {
    
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'F':
        case 'H':
        case 'I':
        case 'J':
        case 'K':
        case 'L':
        case 'N':
        case 'P':
        case 'Q':
        case 'R':
        case 'S':
        case 'T':
        case 'U':
        case 'V':
        case 'W':
        case 'X':
        case 'Y':
        case 'Z':
            vals[code] = value;
            vals_set[code] = true;
            break;
        case 'G':
            handle_g(value);
            break;
        case 'M':
            handle_m(value);
            break;
        }
        
            /*
        case 'X':
            vals[code] = value * unit;
            if (!absolute)
                vals[code] += x;
            dx = vals[code] - x;
            break;
        case 'Y':
            vals[code] = value * unit;
            if (!absolute)
                vals[code] += y;
            dy = vals[code] - y;
            break;
        case 'Z':
            vals[code] = value * unit;
            if (!absolute)
                vals[code] += z;
            dz = vals[code] - z;
            break;
        case 'A':
        case 'F':
        case 'Q':
        case 'R':
        case 'U':
        case 'V':
        case 'W':
            vals[code] = value * unit;
            break;
        case 'G':
            // printf("G%f\n", value);
            handle_g(value);
            break;
        case 'I':
            vals[code] = value * unit;
            if (arc_absolute)
                vals[code] -= x;
            break;
        case 'J':
            vals[code] = value * unit;
            if (arc_absolute)
                vals[code] -= y;
            break;
        case 'K':
            vals[code] = value * unit;
            if (arc_absolute)
                vals[code] -= z;
            break;
        case 'L':
        case 'M':
        case 'T':
            // TODO int
            vals[code] = static_cast<float>(int(value));
            break;
        case 'N':
            break;
        case 'P':
            vals[code] = value;
            break;
        */

    }
    void motionStart(py::list cmds)
    {
        // printf("START!\n");
        for (const auto &c : cmds)
        {
            auto cmd = c.cast<std::string>();
            handleCmd(cmd.c_str());
        }
    }

    float sqr(float x) { return x * x; }

    void motionCenter(float &xc, float &yc, float &zc, float &rval)
    {
        float r = vals['R'];
        if (r > 0.f)
        {
            assert(false);
        }
        else
        {
            xc = x + vals['I'];
            yc = y + vals['J'];
            zc = z + vals['K'];
            rval = std::sqrt(sqr(vals['I']) + sqr(vals['J']) + sqr(vals['K']));
        }
    }

    void push_point(float x, float y, float z, float t, float link)
    {
        cnt++;
        pts.push_back(x);
        pts.push_back(y);
        pts.push_back(z);
        pts.push_back(t);
        pts.push_back(static_cast<float>(block));
        pts.push_back(link);
    }

    py::object motionPath(float line_time)
    {
        switch (gcode)
        {
        case 0:
        case 1: // linear
        {
            float dx = vals['X'] - x;
            float dy = vals['Y'] - y;
            float dz = vals['Z'] - z;
            if (dx != 0.f || dy != 0.f || dz != 0.f)
            {
                float gcode_f = static_cast<float>(gcode);
                push_point(x, y, z, line_time, gcode_f);
                push_point(vals['X'], vals['Y'], vals['Z'], line_time, gcode_f);
            }
        }
        break;

        case 2:
        case 3: // arcs
        {
            push_point(x, y, z, line_time, 1);
            float xval = vals['X'], yval = vals['Y'], zval = vals['Z'];
            float u0, v0, w0, u1, v1, w1;
            float xc, yc, zc, rval;
            motionCenter(xc, yc, zc, rval);
            // printf("xc yc zc %f %f %f %f\n", xc, yc, zc, rval);
            float uc, vc; // TODO
            switch (plane)
            {
            case XY:
                uc = xc;
                vc = yc;
                u0 = x;
                v0 = y;
                w0 = z;
                u1 = xval;
                v1 = yval;
                w1 = zval;
                break;
            case XZ:
                uc = xc;
                vc = zc;
                u0 = x;
                v0 = z;
                w0 = y;
                u1 = xval;
                v1 = zval;
                w1 = yval;
                gcode = 5 - gcode; // flip 2-3 when XZ plane is used
                break;
            case YZ:
                uc = yc;
                vc = zc;
                u0 = y;
                v0 = z;
                w0 = x;
                u1 = yval;
                v1 = zval;
                w1 = xval;
                break;
            }
            float phi0 = std::atan2(v0 - vc, u0 - uc);
            float phi1 = std::atan2(v1 - vc, u1 - uc);
            float saggita = rval == 0.f ? 0.f : (1.f - accuracy / rval);
            float df = saggita > 0.f ? std::min(2.f * std::acos(saggita), pi / 4.f) : pi / 4.f;

            if (gcode == 2)
            {
                if (phi1 >= phi0 - 1e-10f)
                {
                    phi1 -= 2.0f * pi;
                }
                float ws = (w1 - w0) / (phi1 - phi0);
                float phi = phi0 - df;
                while (phi > phi1)
                {
                    float u = uc + rval * std::cos(phi);
                    float v = vc + rval * std::sin(phi);
                    float w = w0 + (phi - phi0) * ws;
                    phi -= df;
                    switch (plane)
                    {
                    case XY:
                        push_point(u, v, w, line_time, 1);
                        break;
                    case XZ:
                        push_point(u, w, v, line_time, 1);
                        break;
                    case YZ:
                        push_point(w, u, v, line_time, 1);
                        break;
                    }
                }
            }
            else
            {
                if (phi1 <= phi0 + 1e-10f)
                {
                    phi1 += 2.0f * pi;
                }
                float ws = (w1 - w0) / (phi1 - phi0);
                float phi = phi0 + df;
                while (phi < phi1)
                {
                    float u = uc + rval * std::cos(phi);
                    float v = vc + rval * std::sin(phi);
                    float w = w0 + (phi - phi0) * ws;
                    phi += df;
                    switch (plane)
                    {
                    case XY:
                        push_point(u, v, w, line_time, 1);
                        break;
                    case XZ:
                        push_point(u, w, v, line_time, 1);
                        break;
                    case YZ:
                        push_point(w, u, v, line_time, 1);
                        break;
                    }
                }
            }
            push_point(xval, yval, zval, line_time, 1);
        }
        break;

        case 4: // dwell
            break;

        case 81:
        case 82:
        case 83:
        case 85:
        case 86:
        case 89:
        {
            float drillz, clearz;
            if(absolute) {
                vals['L'] = 1;
                clearz = retract_z ? std::max(vals['R'], z) : vals['R'];
                drillz = vals['Z'];
            } else {
                clearz = z + vals['R'];
                drillz = clearz + dz;
                float xx = x, yy = y, zz = z; 
                push_point(xx, yy, zz, line_time, 0);
                if(zz != clearz) {
                    zz = clearz;
                    push_point(xx, yy, zz, line_time, 0);
                }
                int reps = std::max(0,int(vals['L']));
                for(int rep=0; rep < reps; rep++) {
                    xx += dx;
                    yy += dx;
                    push_point(xx,yy,zz,line_time,0);
                    if(z > clearz)
                    push_point(xx,yy,clearz,line_time,0);
                    if(z > clearz) {
                        push_point(xx,yy,clearz,line_time,0);
                    }
                    push_point(xx, yy, drillz,line_time,1);
                    zz = clearz;
                    push_point(xx,yy,zz,line_time,0);
                }
            }
        }
        break;
        }
        // return ret;
        return py::none();
    }

    void motionEnd()
    {
        // TODO: this relies on vals['X] ... vals['Z'] never being cleared
        // which i think is pretty weird, but maybe that's how it is suposed
        // to work... but I think this breaks if you do a unit change in 
        // the middle.  or if you do a WCS change ... for that matter I don't
        // see how multiple WCS work at all in bCNC.
        switch (gcode)
        {
        case -1:
            // TODO robust
            // assert(false);
            break;
        case 0:
        case 1:
        case 2:
        case 3:
            x = vals['X'];
            y = vals['Y'];
            z = vals['Z'];
            dx = dy = dz = 0.f;
            if (gcode >= 2)
            {
                vals['R'] = vals['I'] = vals['J'] = vals['K'] = 0.f;
            }
            break;
        case 28:
        case 30:
        case 92:
            vals['X'] = 0.f;
            vals['Y'] = 0.f;
            vals['Z'] = 0.f;
            dx = dy = dz = 0.f;
            break;
        case 81:
        case 82:
        case 83:
            // TODO do 81, 82, 83
            if(absolute) {
                vals['L'] = 1;
            }
            break;
        }
    }

    py::object breakAndSimulate(const char *in)
    {
        std::string s;
        // s.reserve(128);
        const char *i = in;
        while (*i != '\0')
        {
            for (; isalpha(*i) && *i; i++)
                s += *i;
            for (; !isalpha(*i) && *i; i++)
                s += *i;
            // l.append(py::str(s));
            handleCmd(s.c_str());
            s = "";
        }
        execute();
        // TODO(asele): need some way to get good line #
        //py::object ret = motionPath(0);
        py::object ret;
        //motionEnd();
        return ret;
    }

    py::array_t<float> get_numpy()
    {
        py::array_t<float> ret(pts.size());
        // memcpy(ret.data(), pts.data(), sizeof(float) * pts.size());
        return ret;
    }

    void ParseWithExpr(py::str line) {

    }

    // TODO:(aselle) this is fast, but it may be good to have a stop function too. 
    // mode_str -- MINIMAL (just simulate and parse, doesn't return any indiv. gcode lines
    // mode_str -- ACCUMULATE (accumulate all gcode parsed as separate lines)
    // mode_str -- SPLIT (splits the gcode into strings for each block... blocks are created when a positive z mode is encountered)
    py::list ParseBuf(py::str s_py, std::function<std::string(const char *s)> &fallback,
        const std::string& mode_str)
    {
        int n_renumber = 0;
        enum Modes {
            MINIMAL,
            SPLIT,
            ACCUMULATE,
            SIM
        } mode;

        if(mode_str == "MINIMAL") mode = MINIMAL;
        else if(mode_str=="SPLIT") mode = SPLIT;
        else if(mode_str=="ACCUMULATE") mode = ACCUMULATE;
        else if(mode_str=="SIM") mode = SIM;
        else throw std::runtime_error("Invalid mode");

        py::list ret;
        std::string s = s_py.cast<std::string>();
        const char *sb = s.c_str();

        // We use straight character at a time C code here to minimize copies.
        const char *i = sb;

        std::string compact_line;

        bool first_block = true;
        block = 0;

        while (*i != 0)
        {
            compact_line = "";
            // printf("LINE! %x %d\n", *i, *i == 0);
            // Skip initial whitespace.
            for (; *i != 0 && (*i == ' ' || *i == '\t' || *i == '\n' || *i == '\r'); i++)
            {
            }
            const char *line_start = i;

            if (*i == 0)
                break;

            bool fallthrough = false;
            // printf("first char '%c'", *i);
            switch (*i)
            {
            // grbl assignments? fallback
            case '$':
                fallthrough = true;
            // Nothing, so let's return none
                break;
            //case ';':
            // return py::none();
            // Probably variable assignment
            case '_':
                fallthrough = true;
            // Commands
            case '%':
                fallthrough = true;
            }
            bool done = false;
            int paren = 0;
            bool inComment = false;

            // printf("im now here\n");

            for (; *i != 0 && !done; i++)
            {
                switch (*i)
                {
                case '(':
                    paren++;
                    inComment = true;
                    break;
                case ')':
                    paren--;
                    if (paren == 0 && inComment)
                        inComment = false;
                    break;
                case '[':
                    // TODO handle stdexpr
                    if (!inComment)
                    {
                        fallthrough = true;
                    }
                    break;
                case ']':
                    // TODO exception
                    if (!inComment)
                    {
                        fallthrough = true;
                    }
                    break;
                case '=':
                    if (!inComment)
                    {
                        fallthrough = true;
                    }
                case ';':
                    inComment = true;
                    break;
                    // return py::str(fin);
                case ' ':
                    break;
                case '\0':
                case '\n':
                    done = true;
                    break;
                    // default:
                default:
                    if(!inComment) {
                        compact_line += *i;
                    }
                }
                    
            }
            // printf("at! %x -- %x\n", line_start, i);    
            if (fallthrough)
            {
                std::string fall_line(line_start, i);
                // printf("running fallback on '%s'", fall_line.c_str());
                std::string parsed = fallback(fall_line.c_str());
            }
            else
            {
                std::string full_line(line_start, i);
                std::cout<<"-- "<<full_line<<std::endl;
                // printf("found line compact '%s'", compact_line.c_str());
                const char* comp_pristine = compact_line.c_str();
                // TODO(aselle): this N renumbering is ill-advised!
                if(mode==ACCUMULATE) {                    
                    const char* comp = compact_line.c_str();
                    while(*comp != 0) {                        
                        char code = toupper(*comp); 
                        comp++;
                        if(code == 'N') {
                            char* final = 0;
                            float val = std::strtof(comp, &final);
                            comp = final;
                        }
                        break;
                    }
                    std::string renumbered = std::string("N") + std::to_string(n_renumber) + comp;
                    n_renumber += 1;
                    ret.append(renumbered);
                }
                // clear modal
                for(int k=0;k < group_touched.size();k++) group_touched[k] = 0;
                for(int k=0;k < vals_set.size();k++) vals_set[k] = false;
                group_modes[0] = 0;




                const char* comp = comp_pristine;
                while(*comp != 0) {
                    char code = toupper(*comp);
                    comp++;
                    char* final = 0;
                    float val = std::strtof(comp, &final);
                    comp = final;
                    handleCmd(code, val);
                    // printf("Got %c %f", code, val);
                }
                /*
                motionPath(static_cast<float>(_line));
                _line++;
                if(mode == SPLIT && gcode == 0) {
                    if(dz > 0.0 || first_block)  {
                        ret.append(i-sb);
                        first_block = false;
                        block++;
                    } 
                }
                motionEnd();
                */

                execute();
            }
            //if(*i == 0) return py::none();
        }
        return ret;
    }

};

// This takes preprocessed G code strings i.e.
// G21.2F2.12X21.2
// Note no whitespace, exprs, etc.
py::list breakLine(const char *in)
{
    if (!in)
        return py::none();
    const char *i = in;
    py::list l;
    std::string s;
    // s.reserve(128);
    while (*i != '\0')
    {
        for (; isalpha(*i) && *i; i++)
            s += *i;
        for (; !isalpha(*i) && *i; i++)
            s += *i;
        l.append(py::str(s));
        s = "";
    }
    return l;
}

// Parse simple gcode expressions myself... if there is things w/ expressions
// refer to a fallback routine. This makes the fast path very fast and the expression
// path as flexible and simply implemented in python
py::object Parse(const char *s, std::function<py::object(const char *s)> &fallback)
{
    // We use straight character at a time C code here to minimize copies.
    const char *i = s;
    // Skip initial whitespace.
    for (; *i != 0 && (*i == ' ' || *i == '\t'); i++)
        ;
    // printf("c-%x\n", *i);
    switch (*i)
    {
    // grbl assignments? fallback
    case '$':
        return fallback((s));
    // Nothing, so let's return none
    case ';':
        return py::none();
    // Probably variable assignment
    case '_':
        return fallback((s));
    // Commands
    case '%':
        return fallback((s));
    }

    std::string fin;
    fin.reserve(128);

    int paren = 0;
    bool inComment = false;
    // Do a fast path where no expressions are handled.
    // If we encounter anything that looks like an expression abort and
    // go to the slow parser.
    // Note, we need not track much about square brackets (no count) since
    // if we see one outside of a comment we just fallback immediately.
    for (; *i != 0; i++)
    {
        switch (*i)
        {
        case '(':
            paren++;
            inComment = true;
            break;
        case ')':
            paren--;
            if (paren == 0 && inComment)
                inComment = false;
            break;
        case '[':
            // TODO handle stdexpr
            if (!inComment)
            {
                return fallback((s));
            }
            break;
        case ']':
            // TODO exception
            if (!inComment)
            {
                return fallback((s));
            }
            break;
        case '=':
            if (!inComment)
            {
                return fallback((s));
            }
        case ';':
            inComment = true;
            break;
            // return py::str(fin);
        case ' ':
        case '\n':
            break;
        default:
            if (!inComment)
                fin += *i;
        }
    }
    if (fin == "")
        return py::none();
    return py::str(fin);
}

py::list ParseLines(py::list lines, std::function<py::object(const char *s)> &fallback)
{
    py::list ret;

    for (auto o : lines)
    {
        py::str s(o);
        // const char* ss = ;
        ret.append(Parse(s.cast<std::string>().c_str(), fallback));
    }
    return ret;
}



PYBIND11_MODULE(gcodelib, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("Parse", &Parse, "Parse comments out of gcode, fallback to passed function");
    m.def("ParseLines", &ParseLines, "Parse multiple lines of files");
    //  m.def("ParseBuf", &ParseBuf, "Parse multiple lines of files");
    m.def("breakLine", &breakLine, "A function that adds two numbers");

    py::class_<Interpreter>(m, "Interpreter", py::buffer_protocol())
        .def(py::init<>())
        .def("reset", &Interpreter::reset)
        .def("motionStart", &Interpreter::motionStart)
        .def("motionEnd", &Interpreter::motionEnd)
        .def("motionPath", &Interpreter::motionPath)
        .def("breakAndSimulate", &Interpreter::breakAndSimulate)
        .def("ParseBuf", &Interpreter::ParseBuf)
        // .def("numpy", &Interpreter::get_numpy);

        .def_buffer([](Interpreter &m) -> py::buffer_info
                    {
                        return py::buffer_info(
                            m.pts.data(),                           /* Pointer to buffer */
                            sizeof(float),                          /* Size of one scalar */
                            py::format_descriptor<float>::format(), /* Python struct-style format descriptor */
                            2,                                      /* Number of dimensions */
                            {size_t(m.pts.size() / 6), size_t(6)},  /* Buffer dimensions */
                            {sizeof(float) * 6, sizeof(float)});    // strides
                    });
}
