#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

// ======
// Pybind
// ======

namespace py = pybind11;

PYBIND11_MODULE(CentDynReachability, cdr) {
    cdr.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    cdr.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

#ifdef VERSION_INFO
    cdr.attr("__version__") = VERSION_INFO;
#else
    cdr.attr("__version__") = "dev";
#endif
}
