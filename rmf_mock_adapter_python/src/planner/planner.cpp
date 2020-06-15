#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <rmf_mock_adapter/adapter.hpp>

namespace py = pybind11;

// void bind_types(py::module &);

using Plan = rmf_traffic::agv::Plan;
using Planner = rmf_traffic::agv::Planner;

using Start = Planner::Start;
using StartSet = Planner::StartSet;
using Goal = Planner::Goal;
using Options = Planner::Options;
using Configuration = Planner::Configuration;
using Result = Planner::Result;

void bind_plan(py::module &m) {
  // bind_types(m);  // Bound in parent module!
  auto m_plan = m.def_submodule("plan");

  // PLANNER ===================================================================
  py::class_<Start>(m_plan, "Start")
      .def(py::init<rmf_traffic::Time,
                    std::size_t,
                    double,
                    rmf_utils::optional<Eigen::Vector2d>,
                    rmf_utils::optional<std::size_t> >(),
           py::arg("initial_time"),
           py::arg("initial_waypoint"),
           py::arg("initial_orientation"),
           py::arg("location") = rmf_utils::nullopt,
           py::arg("initial_lane") = rmf_utils::nullopt)
      .def_property("time",
                    py::overload_cast<>(&Start::time, py::const_),
                    py::overload_cast<rmf_traffic::Time>(&Start::time))
      .def_property("waypoint",
                    py::overload_cast<>(&Start::waypoint, py::const_),
                    py::overload_cast<std::size_t>(&Start::waypoint))
      .def_property("orientation",
                    py::overload_cast<>(&Start::orientation, py::const_),
                    py::overload_cast<double>(&Start::orientation))
      .def_property("location",
                    py::overload_cast<>(&Start::location, py::const_),
                    py::overload_cast<rmf_utils::optional<Eigen::Vector2d> > \
                        (&Start::location))
      .def_property("lane",
                    py::overload_cast<>(&Start::lane, py::const_),
                    py::overload_cast<rmf_utils::optional<std::size_t> > \
                        (&Start::lane));

  // PLAN ======================================================================
  py::class_<Plan>(m_plan, "Plan")
      // Private constructor
      .def_property_readonly("itinerary",
                             &Plan::get_itinerary)
      .def_property_readonly("waypoints",
                             &Plan::get_waypoints)
      .def_property_readonly("start",
                             &Plan::get_start);

  // WAYPOINT ==================================================================
  py::class_<Plan::Waypoint>(m_plan, "Waypoint")
      // Private constructor
      .def_property_readonly("position",
                             &Plan::Waypoint::position)
      .def_property("time",
                    py::overload_cast<>(&Plan::Waypoint::time, py::const_),
                    py::overload_cast<rmf_traffic::Time>(&Plan::Waypoint::time))
      .def_property_readonly("graph_index",
                             &Plan::Waypoint::graph_index)
      .def_property_readonly("event",
                             &Plan::Waypoint::event);

  // GOAL ======================================================================
  py::class_<Plan::Goal>(m_plan, "Goal")
      .def(py::init<std::size_t>())
      .def(py::init<std::size_t, double>())
      .def_property("waypoint",
                    py::overload_cast<>(&Plan::Goal::waypoint, py::const_),
                    py::overload_cast<std::size_t>(&Plan::Goal::waypoint))
      .def_property("orientation",
                    py::overload_cast<>(&Plan::Goal::orientation, py::const_),
                    py::overload_cast<double>(&Plan::Goal::orientation))
      .def("any_orientation", &Plan::Goal::any_orientation);
}
