#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <angles/angles.h>
#include <cmath>
#include <fstream>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>
#include <gsl_server/algorithms/GrGSL/MovingStateGrGSL.hpp>

namespace GSL
{
    using namespace GrGSL_internal;

    GrGSL::GrGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node)
              IF_GUI(, ui(this))
    {}

    void GrGSL::Initialize()
    {
        Algorithm::Initialize();

#if USE_GUI
        if (!settings.headless)
            ui.run();
#endif
        markers.probabilityMarkers = node->create_publisher<Marker>("probabilityMarkers", 10);
        markers.estimationMarkers = node->create_publisher<Marker>("estimationMarkers", 10);

        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStateGrGSL>(this,
                                                         GrGSLData{
                                                             .node = node,
                                                             .settings = settings,
                                                             .cells = cells,
                                                             .occupancy = occupancy,
                                                             .gridMetadata = gridMetadata,
                                                             .currentRobotPosition = currentRobotPosition,
                                                             .positionOfLastHit = positionOfLastHit});
        stateMachine.forceSetState(waitForMapState.get());
    }

    void GrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        GrGSLLib::GetSettings(node, settings, markers);

        // defaults to <resultsFile without .csv>_declaration.csv
        std::string defaultMetricsFile = "";
        if (resultLogging.resultsFile != "")
        {
            std::string base = resultLogging.resultsFile;
            if (base.size() > 4 && base.substr(base.size() - 4) == ".csv")
                base = base.substr(0, base.size() - 4);
            defaultMetricsFile = base + "_declaration.csv";
        }
        declarationMetricsFile = Utils::getParam<std::string>(node, "declarationMetricsFile", defaultMetricsFile);
    }

    void GrGSL::OnUpdate()
    {
        if (paused)
        {
            GrGSLLib::VisualizeMarkers(
                Grid2D<Cell>(cells, occupancy, gridMetadata),
                markers,
                node,
                settings.colorScaleLimits);
            return;
        }

        Algorithm::OnUpdate();
    }

    void GrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        GrGSLLib::initMetadata(gridMetadata, map, Utils::getParam(node, "scale", 20));
        cells.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        occupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);

        GridUtils::reduceOccupancyMap(map.data, map.info.width, occupancy, gridMetadata);
        GrGSLLib::initializeMap(*this,
                                Grid2D<Cell>(cells, occupancy, gridMetadata));
        positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);
    }

    void GrGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        bool gasHit = concentration > thresholdGas;
        bool significantWind = windSpeed > thresholdWind;

        if (gasHit)
            positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);

        if (gasHit && significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        else if (gasHit)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS BUT NO WIND");
        else if (significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "ONLY WIND");
        else
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING");

        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(cells, occupancy, gridMetadata),
            settings,
            gasHit,
            gasHit ? significantWind : true,
            windDirection,
            positionOfLastHit,
            gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));

        logDeclarationMetrics(gasHit, significantWind);

        movingState->chooseGoalAndMove();
        exploredCells++;
        GrGSLLib::VisualizeMarkers(
            Grid2D<Cell>(cells, occupancy, gridMetadata),
            markers,
            node,
            settings.colorScaleLimits);
    }

    // Four source-declaration metrics of the belief map after each update (see Ojeda2022 Ch.3.B):
    // max cell probability, expected source position and its change, entropy, covariance (trace & determinant).
    // The stop rule (variance < convergence_thr) is untouched; this only writes a CSV row per iteration.
    void GrGSL::logDeclarationMetrics(bool gasHit, bool significantWind)
    {
        if (declarationMetricsFile == "")
            return;

        Grid2D<Cell> grid(cells, occupancy, gridMetadata);
        double sum = 0, maxP = 0, entropy = 0, ex = 0, ey = 0;
        Vector2 maxCoords(0, 0);
        int nFree = 0;
        for (size_t i = 0; i < grid.data.size(); i++)
        {
            if (grid.occupancy[i] != Occupancy::Free)
                continue;
            double p = grid.data[i].sourceProb;
            Vector2 c = grid.metadata.indicesToCoordinates(grid.metadata.indices2D(i));
            nFree++;
            sum += p;
            ex += p * c.x;
            ey += p * c.y;
            if (p > 0)
                entropy -= p * std::log(p);
            if (p > maxP)
            {
                maxP = p;
                maxCoords = c;
            }
        }
        if (sum <= 0)
            return;
        ex /= sum;
        ey /= sum;

        double vxx = 0, vyy = 0, vxy = 0;
        for (size_t i = 0; i < grid.data.size(); i++)
        {
            if (grid.occupancy[i] != Occupancy::Free)
                continue;
            double p = grid.data[i].sourceProb / sum;
            Vector2 c = grid.metadata.indicesToCoordinates(grid.metadata.indices2D(i));
            vxx += p * (c.x - ex) * (c.x - ex);
            vyy += p * (c.y - ey) * (c.y - ey);
            vxy += p * (c.x - ex) * (c.y - ey);
        }

        double change = hasPreviousExpectedSource ? std::hypot(ex - previousExpectedSource.x, ey - previousExpectedSource.y) : -1;
        previousExpectedSource = Vector2(ex, ey);
        hasPreviousExpectedSource = true;

        std::ifstream probe(declarationMetricsFile);
        bool writeHeader = !probe.good() || probe.peek() == std::ifstream::traits_type::eof();
        probe.close();

        std::ofstream file(declarationMetricsFile, std::ios_base::app);
        if (!file.is_open())
        {
            GSL_WARN("Unable to open declaration metrics file at: {}", declarationMetricsFile);
            return;
        }
        if (writeHeader)
            file << "run_start,iteration,t,robot_x,robot_y,gas_hit,wind_present,max_p,max_x,max_y,exp_x,exp_y,exp_change,"
                    "entropy,var_x,var_y,cov_xy,cov_trace,cov_det,n_free,sum_p,convergence_thr\n";
        file << fmt::format("{:.3f},{},{:.3f},{:.4f},{:.4f},{},{},{:.6g},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.6g},{:.6g},{:.6g},{:.6g},{:.6g},{:.6g},{},{:.6g},{}\n",
                            startTime.seconds(), exploredCells + 1, (node->now() - startTime).seconds(),
                            currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y,
                            (int)gasHit, (int)significantWind,
                            maxP, maxCoords.x, maxCoords.y, ex, ey, change,
                            entropy, vxx, vyy, vxy, vxx + vyy, vxx * vyy - vxy * vxy, nFree, sum, settings.convergence_thr);
    }

    double GrGSL::probability(const Vector2Int& indices) const
    {
        size_t index = gridMetadata.indexOf(indices);
        return cells[index].sourceProb;
    }

    GSLResult GrGSL::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get() || stateMachine.getCurrentState() == waitForGasState.get())
            return GSLResult::Running;
        Grid2D<Cell> grid(cells, occupancy, gridMetadata);
        rclcpp::Duration time_spent = node->now() - startTime;
        if (time_spent.seconds() > resultLogging.maxSearchTime)
        {
            saveResultsToFile(GSLResult::Failure);
            return GSLResult::Failure;
        }

        if (resultLogging.navigationTime == -1)
        {
            if (sqrt(pow(currentRobotPose.pose.pose.position.x - resultLogging.sourcePositionGT.x, 2) +
                     pow(currentRobotPose.pose.pose.position.y - resultLogging.sourcePositionGT.y, 2)) < 0.5)
            {
                resultLogging.navigationTime = time_spent.seconds();
            }
        }

        double variance = GrGSLLib::varianceSourcePosition(grid);
        GSL_INFO("Variance: {}", variance);

        if (variance < settings.convergence_thr)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    void GrGSL::saveResultsToFile(GSLResult result)
    {
        Grid2D<Cell> grid(cells, occupancy, gridMetadata);
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();

        Vector2 sourceLocationAll = GrGSLLib::expectedValueSource(grid, 1);
        Vector2 sourceLocation = GrGSLLib::expectedValueSource(grid, 0.05);

        double error = sqrt(pow(resultLogging.sourcePositionGT.x - sourceLocation.x, 2) + pow(resultLogging.sourcePositionGT.y - sourceLocation.y, 2));
        double errorAll = sqrt(pow(resultLogging.sourcePositionGT.x - sourceLocationAll.x, 2) + pow(resultLogging.sourcePositionGT.y - sourceLocationAll.y, 2));

        std::string resultString = fmt::format("RESULT IS: Success={}, Search_t={:.2f}, Error={:.2f}", (int)result, search_t, error);
        GSL_INFO_COLOR(fmt::terminal_color::blue, "{}", resultString);

        // Save to file
        if (resultLogging.resultsFile != "")
        {
            std::ofstream file;
            file.open(resultLogging.resultsFile, std::ios_base::app);
            if (result != GSLResult::Success)
                file << "FAILED ";

            file << resultLogging.navigationTime << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " "
                 << GrGSLLib::varianceSourcePosition(grid) << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging result. Skipping it.");

        if (resultLogging.navigationPathFile != "")
        {
            std::ofstream file;
            file.open(resultLogging.navigationPathFile, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped p : resultLogging.robotPosesVector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging path. Skipping it.");
    }

} // namespace GSL