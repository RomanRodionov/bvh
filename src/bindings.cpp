#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "bvh.h"

namespace py = pybind11;

PYBIND11_MODULE(bvh, m) {
    py::class_<BVH>(m, "BVH")
        .def(py::init<>())
        .def("load_scene", &BVH::load_scene)
        .def("build_bvh", &BVH::build_bvh)
        .def("save_as_obj", &BVH::save_as_obj)
        .def("intersect_leaves", [](BVH& self, py::array_t<float> ray_origins, py::array_t<float> ray_directions) {
            auto r_o = ray_origins.unchecked<2>();
            auto r_d = ray_directions.unchecked<2>();

            if (r_o.shape(0) != r_d.shape(0)) {
                throw std::runtime_error("Mismatched ray origins and directions!");
            }

            std::vector<glm::vec3> origins(r_o.shape(0)), directions(r_d.shape(0));
            for (ssize_t i = 0; i < r_o.shape(0); ++i) {
                origins[i] = glm::vec3(r_o(i, 0), r_o(i, 1), r_o(i, 2));
                directions[i] = glm::vec3(r_d(i, 0), r_d(i, 1), r_d(i, 2));
            }

            int size = origins.size();
            py::array_t<float> output({2, size});
            auto buf = output.mutable_unchecked<2>();
            for (size_t i = 0; i < origins.size(); ++i) {
                auto [x, y] = self.intersect_leaves(origins[i], directions[i]);
                buf(0, i) = x;
                buf(1, i) = y;
            }

            return output;
        });
}
