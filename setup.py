# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.    

from glob import glob
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

__version__ = "0.0.1"

#print("MODULES ", sorted(glob("src/gcodelib/parser.cpp")))

ext_modules = [
    Pybind11Extension(
        "tridexel",
        sorted(glob("src/tridexel/tridexel_pymodule.cpp")),  # Sort source files for reproducibility
        extra_compile_args = ["-O3","-march=native","-g", "-DNDEBUG"], 
	#extra_compile_args = ["/O3","/arch:AVX2"] 
    ),
    Pybind11Extension(
        "gcodelib",
        sorted(glob("src/gcodelib/parser.cpp")),  # Sort source files for reproducibility
        extra_compile_args = ["-O3","-march=native","-g", "-DNDEBUG"], 
	#extra_compile_args = ["/O3","/arch:AVX2"] 
    ),
]



setup(
    name="subtractosim",
    version=__version__,
    author="Andrew SelLe",
    author_email="aselle@andyselle.com",
    description="Subtracto sim",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)

