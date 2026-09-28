#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <carb/BindingsUtils.h>
#include <carb/Framework.h>

#include <omni/anim/navigation/INavigation.h>

#include <stdexcept>
#include <string>
#include <vector>

CARB_BINDINGS("a_pipeline.isaac5_native_controller", "python")

namespace py = pybind11;
namespace nav = omni::anim::navigation;

namespace
{

std::vector<carb::Float3> toFloat3(const std::vector<std::vector<float>>& values, const char* name)
{
    std::vector<carb::Float3> result;
    result.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i].size() != 3)
        {
            throw std::invalid_argument(std::string(name) + "[" + std::to_string(i) + "] must have three values");
        }
        result.push_back({ values[i][0], values[i][1], values[i][2] });
    }
    return result;
}

void requireCount(size_t expected, size_t actual, const char* name)
{
    if (expected != actual)
    {
        throw std::invalid_argument(
            std::string(name) + " has " + std::to_string(actual) + " entries; expected " +
            std::to_string(expected));
    }
}

class NativeController
{
public:
    NativeController(float obstaclePadding, float passagePadding, float agentPadding, float agentPaddingMinVelocity)
    {
        auto* framework = carb::getFramework();
        if (!framework)
        {
            throw std::runtime_error("Carbonite framework is unavailable");
        }
        navigation_ = framework->acquireInterface<nav::INavigation>();
        if (!navigation_)
        {
            throw std::runtime_error("omni.anim.navigation INavigation is unavailable; enable the navigation extension first");
        }
        navMesh_ = navigation_->getNavMesh();
        if (!navMesh_)
        {
            throw std::runtime_error("no baked NavMesh is available");
        }

        nav::INavController::ControllerParams params;
        params.obstaclePadding = obstaclePadding;
        params.passagePadding = passagePadding;
        params.agentPadding = agentPadding;
        params.agentPaddingMinVelocity = agentPaddingMinVelocity;
        controller_ = navMesh_->createController(params);
        if (!controller_)
        {
            throw std::runtime_error("INavMesh::createController returned null");
        }
    }

    bool createAgents(
        const std::vector<int>& ids,
        const std::vector<std::vector<float>>& positions,
        const std::vector<float>& speeds,
        const std::vector<float>& radii,
        const std::vector<float>& heights)
    {
        const size_t count = ids.size();
        requireCount(count, positions.size(), "positions");
        requireCount(count, speeds.size(), "speeds");
        requireCount(count, radii.size(), "radii");
        requireCount(count, heights.size(), "heights");
        auto points = toFloat3(positions, "positions");
        return controller_->createAgents(
            ids.data(), static_cast<int>(count), points.data(), speeds.data(), radii.data(), heights.data());
    }

    bool updateAgents(
        const std::vector<int>& ids,
        const py::object& positions,
        const py::object& speeds,
        const py::object& radii,
        const py::object& heights,
        const py::object& goals)
    {
        const size_t count = ids.size();
        std::vector<carb::Float3> positionValues;
        std::vector<float> speedValues;
        std::vector<float> radiusValues;
        std::vector<float> heightValues;
        std::vector<carb::Float3> goalValues;

        if (!positions.is_none())
        {
            auto values = positions.cast<std::vector<std::vector<float>>>();
            requireCount(count, values.size(), "positions");
            positionValues = toFloat3(values, "positions");
        }
        if (!speeds.is_none())
        {
            speedValues = speeds.cast<std::vector<float>>();
            requireCount(count, speedValues.size(), "speeds");
        }
        if (!radii.is_none())
        {
            radiusValues = radii.cast<std::vector<float>>();
            requireCount(count, radiusValues.size(), "radii");
        }
        if (!heights.is_none())
        {
            heightValues = heights.cast<std::vector<float>>();
            requireCount(count, heightValues.size(), "heights");
        }
        if (!goals.is_none())
        {
            auto values = goals.cast<std::vector<std::vector<float>>>();
            requireCount(count, values.size(), "goals");
            goalValues = toFloat3(values, "goals");
        }

        return controller_->updateAgents(
            ids.data(), static_cast<int>(count),
            positionValues.empty() ? nullptr : positionValues.data(),
            speedValues.empty() ? nullptr : speedValues.data(),
            radiusValues.empty() ? nullptr : radiusValues.data(),
            heightValues.empty() ? nullptr : heightValues.data(),
            goalValues.empty() ? nullptr : goalValues.data(), nullptr);
    }

    void simulate(float dt)
    {
        if (!(dt > 0.0f))
        {
            throw std::invalid_argument("dt must be positive");
        }
        controller_->simulateAgents(dt);
    }

    std::vector<std::vector<float>> simulated(const std::vector<int>& ids) const
    {
        nav::Vec3Array points;
        controller_->getSimulatedAgents(ids.data(), static_cast<int>(ids.size()), points);
        if (points.size() != ids.size())
        {
            throw std::runtime_error(
                "getSimulatedAgents returned " + std::to_string(points.size()) + " entries; expected " +
                std::to_string(ids.size()));
        }
        std::vector<std::vector<float>> result;
        result.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            result.push_back({ points[i].x, points[i].y, points[i].z });
        }
        return result;
    }

    std::vector<std::vector<float>> pathPoints(int id) const
    {
        auto path = controller_->getAgentPath(id);
        std::vector<std::vector<float>> result;
        if (!path)
        {
            return result;
        }
        const auto* points = path->getPoints();
        result.reserve(path->getPointCount());
        for (size_t i = 0; i < path->getPointCount(); ++i)
        {
            result.push_back({ points[i].x, points[i].y, points[i].z });
        }
        return result;
    }

    py::tuple closestPoint(
        const std::vector<float>& target,
        int searchIslandId,
        float agentRadius,
        float agentHeight) const
    {
        if (target.size() != 3)
        {
            throw std::invalid_argument("target must have three values");
        }
        const carb::Float3 requested{ target[0], target[1], target[2] };
        carb::Float3 point{};
        int foundIslandId = -1;
        const bool found = navMesh_->queryClosestPoint(
            requested, &point, nullptr, 0, agentRadius, agentHeight,
            searchIslandId, &foundIslandId);
        return py::make_tuple(
            found,
            std::vector<float>{ point.x, point.y, point.z },
            foundIslandId);
    }

    bool destroyAgents(const std::vector<int>& ids)
    {
        return controller_->destroyAgents(ids.data(), static_cast<int>(ids.size()));
    }

private:
    nav::INavigation* navigation_ = nullptr;
    nav::INavMeshPtr navMesh_;
    nav::INavControllerPtr controller_;
};

bool nativeControllerAvailable()
{
    auto* framework = carb::getFramework();
    if (!framework)
    {
        return false;
    }
    auto* navigation = framework->acquireInterface<nav::INavigation>();
    return navigation && navigation->getNavMesh();
}

} // namespace

PYBIND11_MODULE(_isaac5_native_controller, module)
{
    module.doc() = "Minimal Isaac Sim 5.1 INavController binding for a_pipeline";
    module.def("available", &nativeControllerAvailable);
    py::class_<NativeController>(module, "NativeController")
        .def(
            py::init<float, float, float, float>(),
            py::arg("obstacle_padding") = 0.1f,
            py::arg("passage_padding") = 1.0f,
            py::arg("agent_padding") = 0.5f,
            py::arg("agent_padding_min_velocity") = 0.1f)
        .def("create_agents", &NativeController::createAgents)
        .def(
            "update_agents", &NativeController::updateAgents,
            py::arg("ids"), py::arg("positions") = py::none(), py::arg("speeds") = py::none(),
            py::arg("radii") = py::none(), py::arg("heights") = py::none(), py::arg("goals") = py::none())
        .def("simulate", &NativeController::simulate)
        .def("simulated", &NativeController::simulated)
        .def("path_points", &NativeController::pathPoints)
        .def(
            "closest_point", &NativeController::closestPoint,
            py::arg("target"), py::arg("search_island_id") = -1,
            py::arg("agent_radius") = 0.25f, py::arg("agent_height") = 1.7f)
        .def("destroy_agents", &NativeController::destroyAgents);
}
