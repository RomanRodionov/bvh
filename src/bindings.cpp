#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <tuple>

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
                throw std::runtime_error("Mismatched ray origins and directigons!");
            }

            std::vector<glm::vec3> origins(r_o.shape(0)), directions(r_d.shape(0));
            for (ssize_t i = 0; i < r_o.shape(0); ++i) {
                origins[i] = glm::vec3(r_o(i, 0), r_o(i, 1), r_o(i, 2));
                directions[i] = glm::vec3(r_d(i, 0), r_d(i, 1), r_d(i, 2));
            }

            int size = origins.size();
            py::array_t<bool> mask({size});
            py::array_t<int> leaf_indices({size});
            py::array_t<float> t_enters({size});
            py::array_t<float> t_exits({size});

            auto mask_buf = mask.mutable_unchecked<1>();
            auto leaf_indices_buf = leaf_indices.mutable_unchecked<1>();
            auto t_enters_buf = t_enters.mutable_unchecked<1>();
            auto t_exits_buf = t_exits.mutable_unchecked<1>();

            for (int i = 0; i < size; ++i) {
                auto [mask, leaf_index, t_enter, t_exit] = self.intersect_leaves(origins[i], directions[i]);
                mask_buf(i) = mask;
                leaf_indices_buf(i) = leaf_index;
                t_enters_buf(i) = t_enter;
                t_exits_buf(i) = t_exit;
            }

            return std::make_tuple(mask, leaf_indices, t_enters, t_exits);
        })
        .def("depth", py::overload_cast<>(&BVH::depth))
        .def("n_nodes", &BVH::n_nodes)
        .def("n_leaves", py::overload_cast<>(&BVH::n_leaves));
}
