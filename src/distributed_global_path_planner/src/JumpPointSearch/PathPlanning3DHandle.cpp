#include "PathPlanning3DHandle.h"

namespace pathplanning
{

PathPlanning3DHandle::PathPlanning3DHandle()
: m_mapUtilPtr(std::make_shared<JPS::VoxelMapUtil>())
, m_plannerJPSPtr(new JPSPlanner3D(false))
, m_plannerDMPtr(new DMPlanner3D(false))
{

}

void
PathPlanning3DHandle::setParams(const PathPlanningParams& params)
{
    m_params = params;

    correctResolutionOffset();

    loadFreespace(m_params.mapResolution,
                  m_params.mapMinBoundsX, m_params.mapMaxBoundsX,
                  m_params.mapMinBoundsY, m_params.mapMaxBoundsY,
                  m_params.mapMinBoundsZ, m_params.mapMaxBoundsZ);

    generateRobotModel(m_robotModel);

    m_plannerJPSPtr->setMapUtil(m_mapUtilPtr);

    m_plannerDMPtr->setPotentialRadius(Vec3f(m_params.potentialRadius, m_params.potentialRadius, m_params.potentialRadius));
    m_plannerDMPtr->setSearchRadius(Vec3f(m_params.searchRadius, m_params.searchRadius, m_params.searchRadius));
    // the position in setMap is the center point for the PotentialMapRange
    // however if the PotentialMapRange is 0 (which is the default value)
    // the entire map will have the potential field around the obs so the position in setMap does not affect anything
    m_plannerDMPtr->setMap(m_mapUtilPtr, Vec3f((m_params.mapMinBoundsX + m_params.mapMaxBoundsX)/2,
                                               (m_params.mapMinBoundsY + m_params.mapMaxBoundsY)/2,
                                               (m_params.mapMinBoundsZ + m_params.mapMaxBoundsZ)/2));
}

void
PathPlanning3DHandle::resetMap()
{
    std::fill(m_mapUtilPtr->map_.begin(), m_mapUtilPtr->map_.end(), 0);
}

void
PathPlanning3DHandle::correctResolutionOffset()
{
    //to correct resolution offset
    m_params.mapMinBoundsX -= m_params.mapResolution/2.0;
    m_params.mapMaxBoundsX += m_params.mapResolution/2.0;
    m_params.mapMinBoundsY -= m_params.mapResolution/2.0;
    m_params.mapMaxBoundsY += m_params.mapResolution/2.0;
    m_params.mapMinBoundsZ -= m_params.mapResolution/2.0;
    m_params.mapMaxBoundsZ += m_params.mapResolution/2.0;
}

void
PathPlanning3DHandle::plan(const std::vector<Eigen::Vector3d>& inputWaypoints, std::vector<Eigen::Vector3d>& outputWaypoints)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    outputWaypoints.clear();

    bool planSuccess;

    m_plannerJPSPtr->updateMap();

    // the position in setMap is the center point for the PotentialMapRange
    // however if the PotentialMapRange is 0 (which is the default value)
    // the entire map will have the potential field around the obs so the position in setMap does not affect anything
    m_plannerDMPtr->setMap(m_mapUtilPtr, Vec3f((m_params.mapMinBoundsX + m_params.mapMaxBoundsX)/2,
                                               (m_params.mapMinBoundsY + m_params.mapMaxBoundsY)/2,
                                               (m_params.mapMinBoundsZ + m_params.mapMaxBoundsZ)/2));

    std::vector<Eigen::Vector3d> plannedPath;
    for (int i = 1; i < inputWaypoints.size(); i++)
    {
        const Vec3f s(inputWaypoints.at(i-1).x(), inputWaypoints.at(i-1).y(), inputWaypoints.at(i-1).z());
        const Vec3f g(inputWaypoints.at(i).x(), inputWaypoints.at(i).y(), inputWaypoints.at(i).z());

        //jps or A* path plan
        planSuccess = m_plannerJPSPtr->plan(s, g, m_params.hCostWeight, m_params.performJPS);

        if (!planSuccess)
        {
            break;
        }

        auto path_jps = m_plannerJPSPtr->getRawPath();

        //use artificial field to push path
        if (m_params.potentialRadius>0.0 && m_params.searchRadius>0.0)
        {
            planSuccess = m_plannerDMPtr->computePath(s, g, path_jps);

            if (!planSuccess)
            {
                break;
            }

            path_jps = m_plannerDMPtr->getRawPath();
        }

        if (!plannedPath.empty())
        {
            plannedPath.pop_back();
        }

        for (auto&& pathPt : path_jps)
        {
            Eigen::Vector3d pt (pathPt(0), pathPt(1), pathPt(2));
            plannedPath.push_back(pt);
        }
    }

    prunePath(plannedPath, outputWaypoints);

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    std::cout << "pure planning time: " << elapsed_time_ms << "ms" << std::endl;
}

void
PathPlanning3DHandle::plan(const std::vector<Eigen::Vector3d>& inputWaypoints, const bool& useJPS, std::vector<Eigen::Vector3d>& outputWaypoints)
{
    bool tempPerformJPS = m_params.performJPS;
    m_params.performJPS = useJPS;

    plan(inputWaypoints, outputWaypoints);

    m_params.performJPS = tempPerformJPS;
}

bool
PathPlanning3DHandle::collisionFoundBetween2Points(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, Eigen::Vector3d& pointBeforeCollision)
{
    pointBeforeCollision = start;

    auto pns = m_mapUtilPtr->rayTrace(start, goal);

    for (const auto &pn : pns)
    {
        if (m_mapUtilPtr->map_[m_mapUtilPtr->getIndex(pn)] >= 1)
        {
            return true;
        }
        else
        {
            pointBeforeCollision = m_mapUtilPtr->intToFloat(pn);
        }
    }

    pointBeforeCollision = goal;

    return false;
}

void
PathPlanning3DHandle::prunePath(const std::vector<Eigen::Vector3d>& oldPath, std::vector<Eigen::Vector3d>& newPath)
{
    newPath.clear();

    if (oldPath.size() < 2)
    {
        newPath=oldPath;
        return;
    }

    Eigen::Vector3d s = oldPath.at(0);
    newPath.push_back(s);
    Eigen::Vector3d g = oldPath.at(1);
    Eigen::Vector3d direction = g-s;
    direction.normalize();
    for (int i = 2; i < oldPath.size(); i++)
    {
        g = oldPath.at(i);
        Eigen::Vector3d validateVec = g-s;
        validateVec.normalize();

        double dotProduct = direction.dot(validateVec);

        //add prv point when direction changes
        if (dotProduct<(1.0-FLT_EPSILON) || dotProduct>(1.0+FLT_EPSILON))
        {
            s = oldPath.at(i-1);
            newPath.push_back(s);
            direction = g-s;
            direction.normalize();
        }
    }

    newPath.push_back(oldPath.back());
}

void
PathPlanning3DHandle::generateRobotModel(std::vector<Eigen::Vector3d>& points)
{
    // Determine height increment
    int numPointsXY = std::ceil(2.0 * m_params.radius / (0.5 * m_params.mapResolution));
    double incXY = 2.0 * m_params.radius / numPointsXY;

    // Determine height increment
    int numPointsZ = std::ceil(m_params.cylinderHeight / (0.5 * m_params.mapResolution));
    double incZ = m_params.cylinderHeight / numPointsZ;

    // Generate cylindrical robot model
    for (double x = -m_params.radius; x <= m_params.radius; x += incXY)
    {
        for (double y = -m_params.radius; y <= m_params.radius; y += incXY)
        {
            // Point lies within base circle
            if (x * x + y * y <= m_params.radius * m_params.radius)
            {
                for (double z = -0.5 * m_params.cylinderHeight; z <= 0.5 * m_params.cylinderHeight; z += incZ)
                {
                    points.push_back(Eigen::Vector3d(x, y, z));
                }
            }
        }
    }
}

void
PathPlanning3DHandle::loadFreespace(const double mapRes,
                                    const double minBoundX, const double maxBoundX,
                                    const double minBoundY, const double maxBoundY,
                                    const double minBoundZ, const double maxBoundZ)
{
    Vecf<3> ori(minBoundX, minBoundY, minBoundZ);

    Veci<3> dim(static_cast<int>(std::ceil((maxBoundX-minBoundX) / mapRes)),
                static_cast<int>(std::ceil((maxBoundY-minBoundY) / mapRes)),
                static_cast<int>(std::ceil((maxBoundZ-minBoundZ) / mapRes)));

    std::vector<signed char> data; // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y * z;
    data.resize(dim[0] * dim[1] * dim[2], 0); // initialize as free map, free cell has 0 occupancy

    m_mapUtilPtr->setMap(ori, dim, data, mapRes);
}

void
PathPlanning3DHandle::addInflatedObstaclesPoints(const sensor_msgs::PointCloud& pointCloud)
{
    signed char occupied = 1;

    for (auto&& point : pointCloud.points)
    {
        // Add inflated point to map
        for (const Eigen::Vector3d& robotPoint : m_robotModel)
        {
            Vecf<3> obsPt(point.x + robotPoint.x(),
                          point.y + robotPoint.y(),
                          point.z + robotPoint.z());

            auto pn = m_mapUtilPtr->floatToInt(obsPt);

            if (!m_mapUtilPtr->isOutside(pn))
            {
                auto idx = m_mapUtilPtr->getIndex(pn);
                m_mapUtilPtr->map_.at(idx) = occupied;
            }
        }
    }
}

void
PathPlanning3DHandle::addObstaclesPoints(const sensor_msgs::PointCloud& pointCloud)
{
    signed char occupied = 1;

    for (auto&& point : pointCloud.points)
    {
        Vecf<3> obsPt(point.x, point.y, point.z);

        auto pn = m_mapUtilPtr->floatToInt(obsPt);

        if (!m_mapUtilPtr->isOutside(pn))
        {
            auto idx = m_mapUtilPtr->getIndex(pn);
            m_mapUtilPtr->map_.at(idx) = occupied;
        }
    }
}

} // namespace pathplanning