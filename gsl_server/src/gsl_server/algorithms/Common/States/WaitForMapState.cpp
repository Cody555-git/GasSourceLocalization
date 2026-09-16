#include <gsl_server/algorithms/Common/States/WaitForMapState.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>

namespace GSL
{

    void WaitForMapState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering WaitForMap");
        GSL_TRACE("Need occupancy map and costmap");
        using namespace std::placeholders;

        mapSub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(algorithm->getParam<std::string>("map_topic", "map"),
                 rclcpp::QoS(1).reliable().transient_local(),
                 std::bind(&WaitForMapState::mapCallback, this, _1));
        costmapSub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                         algorithm->getParam<std::string>("costmap_topic", "global_costmap/costmap"), 1, std::bind(&WaitForMapState::costmapCallback, this, _1));
    }

    void WaitForMapState::OnExitState(State* previous)
    {
        algorithm->startTime = algorithm->node->now();
    }

    void WaitForMapState::mapCallback(OccupancyGrid::SharedPtr msg)
    {
        GSL_TRACE("Got occupancy map");
        algorithm->onGetMap(msg);
        hasMap = true;
        // Resetting mapSub here (inside the subscription's own callback, while rclcpp::spin_some
        // is still iterating the executor's entities) causes a double free in
        // rclcpp::Executor::remove_node when spin_some tears down its temporary executor
        // (observed 2026-09-16: SIGABRT "double free or corruption (out)" right after this
        // callback ran). Deferring the reset to Algorithm::OnUpdate's functionQueue.run(),
        // which executes after spin_some has returned, avoids destroying the subscription
        // mid-callback.
        algorithm->functionQueue.submit([this]() { mapSub = nullptr; });
        if (hasCostmap)
            setNextState();
    }

    void WaitForMapState::costmapCallback(OccupancyGrid::SharedPtr msg)
    {
        GSL_TRACE("Got cost map");
        algorithm->onGetCostMap(msg);
        hasCostmap = true;
        algorithm->functionQueue.submit([this]() { costmapSub = nullptr; });
        if (hasMap)
            setNextState();
    }

    void WaitForMapState::setNextState()
    {
        if (shouldWaitForGas)
            algorithm->stateMachine.forceSetState(algorithm->waitForGasState.get());
        else
            algorithm->stateMachine.forceSetState(algorithm->stopAndMeasureState.get());
    }
} // namespace GSL

#if USE_GUI
#include "imgui.h"
void GSL::WaitForMapState::RenderUI()
{
    ImGui::Text("WaitForMapState");
}
#endif