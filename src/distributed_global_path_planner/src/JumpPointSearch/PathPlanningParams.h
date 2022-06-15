#pragma once

namespace pathplanning
{
    struct PathPlanningParams
    {
        PathPlanningParams()
        : radius(0.5)
        , cylinderHeight(1.0)
        , mapResolution(1.0)
        , mapMinBoundsX(-150.0)
        , mapMaxBoundsX(150.0)
        , mapMinBoundsY(-150.0)
        , mapMaxBoundsY(150.0)
        , mapMinBoundsZ(1.0)
        , mapMaxBoundsZ(2.0)
        , hCostWeight(1.0)
        , potentialRadius(3.0)
        , searchRadius(1.5)
        , performJPS(true)
        {}
        
        // Map parameters
        double radius;          // radius in XY-plane of robot cylinder model (in m)
        double cylinderHeight;  // height (Z) of robot cylinder model (in m)
        double mapResolution;   // desired map resolution (in m)
        double mapMinBoundsX;   // desired min bound for x dimension (in m)
        double mapMaxBoundsX;   // desired max bound for x dimension (in m)
        double mapMinBoundsY;   // desired min bound for y dimension (in m)
        double mapMaxBoundsY;   // desired max bound for y dimension (in m)
        double mapMinBoundsZ;   // desired min bound for z dimension (in m)
        double mapMaxBoundsZ;   // desired max bound for z dimension (in m)

        // Planner parameters
        double hCostWeight;     // multiplier to h-cost for more directed expansion of nodes
        double potentialRadius; // radius of potential field around obstacle
        double searchRadius;    // radius of region to search around given path

        bool performJPS;
	};
} // namespace pathplanning