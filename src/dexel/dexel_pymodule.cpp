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
#include "dexel.h"

#include <stdint.h>
// Get ssize_t defined
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

namespace py = pybind11;


int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(tridexel, m) {
    m.doc() = "TriDexel Module"; 

    py::class_<V3f>(m, "V3f")
        .def(py::init<float,float,float>())
        .def_readwrite("x", &V3f::x)
        .def_readwrite("y", &V3f::y)
        .def_readwrite("z", &V3f::z)
        .def("dot", &V3f::dot);

    py::class_<Traceable>(m, "Traceable");

    py::class_<Sphere, Traceable>(m, "Sphere")
        .def(py::init<V3f, float, int>())
        .def("box", &Sphere::box);

    py::class_<Cylinder, Traceable>(m, "Cylinder")
        .def(py::init<V3f, float, float, int>())
        .def("box", &Cylinder::box);

    py::class_<Box, Traceable>(m, "Box")
        .def(py::init<V3f, V3f, int>())
        .def("box", &Box::box);


    py::class_<Box3f>(m, "Box3f")
        .def(py::init<V3f,V3f>());

    // m.def("add", &add, "A function that adds two numbers");
    py::class_<TriDexelImage>(m, "TriDexelImage")
        .def(py::init<float, Box3f>())
        .def("trace", &TriDexelImage::trace)
        .def("save_obj", &TriDexelImage::save_obj)
        .def("get_pointcloud", &TriDexelImage::get_pointcloud,  py::return_value_policy::reference)
        .def("inplace_cut", &TriDexelImage::inplace_cut)
        .def("inplace_union", &TriDexelImage::inplace_union)
    ;


    py::class_<PointCloud>(m, "PointCloud", py::buffer_protocol())
        .def(py::init<>())
        .def_buffer([](PointCloud& pc) -> py::buffer_info {
            //return py::buffer_info((void*)0, sizeof(float), py::format_descriptor<float>::format(), 2, {0}, {sizeof(float)}, true);
            const std::vector<Point>& points = *(pc.points);
            constexpr size_t elements = sizeof(Point) / sizeof(Point::scalar_type);
            ssize_t woot;
            ssize_t shape[] = {(ssize_t)points.size(), elements};
            using scalar = Point::scalar_type;
            // std::cout<<points.data()<<std::endl;
            return py::buffer_info(
                 (void*)points.data(),
                 sizeof(scalar),
                 py::format_descriptor<scalar>::format(),
                 2,
                 shape,
                 {sizeof(Point), sizeof(scalar)},
                 true
             );
        });
    ;


}



