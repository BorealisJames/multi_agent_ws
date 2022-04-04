# Quick info about the src files

# Phase Syncronizer is the state manager

- Each Drones has 
- The Phase Synchronizer is the State Manager that ensures all the in the same Phase
    Phase 1: Get poses from the follower drones that form the convex region
    Phase 2: From the average extrama of the Convex region and the Human position, obtain the agent own utility and angle. 
    Then publish it and obtain from other drones their utility and angle. 
    After obtaining other agent's utility for angle calculate consensus in direction.
    Phase 3: Process own pointcloud data (add human position + screen ard human, removing points near ground). 
    Generate free spce convex region from Point (Average extrema position of convex hull of drone) and from Path (Average extrema of convex hull of drone position + goal position) and from point cloud data using DecompUtil. 
    Intersect the Convex region generated from Point and Path.
    Publish this obstacle free region and obtain other drones convex free region.
    Normalize the values(?)
    Do optimization with the normalized values, find positions in the convex region, given constraints of agent radius, desired distance in formation, goal position, and stuff. Now formation position is already generated on the drones.
    Start assignment problem, ie which drone go to which position.
    Publish the positions to go.
    Phase 4: assignment of formation position.
    Publish again own drone formation position to go and double check with incoming other agent data to check all agent assignments are the same.
    Phase 5: Check if any agent is still on Phase 4. If there is Publish own agent's task (position) and position..

# Convex Hull of Robot Position 

- A class that takes in 2 matrix arrays, input and output. From the input matrix, it does something to the output matrix that i has honestly no idea whats gg on, my hear hurts lmao later i will decipher

# Direction of motion

- Responsible for obtaining goal to go to from the human system pose. A goal will not be generated if the Human hasn't move a significant amount (sort of stationary)
- Find the direction of motion for the robot to go to.
- In 3D, elevation angle is considered, otherwise its only azimuth angle
- A utility is assigned to every index of angle considered ??
- Agents own vector of angle utility and index is genereated, in Enter of Phase 2

# ProcessPointCloud

- This class contains helper functions remove points within a certain radius / boudning box from the human from the point cloud
- Also contains helper funcs to append to point cloud

# Generate convex region

- Contains all the helper functions that Generates convex region from Point and Path. 
- Uses the Decomp util tools (convex decomposition of a free space in a cluttered environment, to put it simply, generates convex hull of free space from a pointcloud environment)
- Go to catkin_ws/DecompROS there are examples there and for examples there.
- In 3D free space generated is a Polyhedron and ellipsoid, in 2D free space generated is ellipse and polygon.
- Path Convex region generation (from a sequence of points):
    - Get point cloud and Path data
    - Generate Polyhedron around the path 
    - From Polyhedron Convex Hull "simplyfy" to Ellipsoid with some magical math tricks and finding intersection of ellipsoid with obstacles 
    - Ellipsoid space is now the free space considered.
- For Point Convex region generation, its similar to path except for the Polyhedron is inflated from the specified Point. There is a bit of mathematical trick in addition to it too. 

# VirtualPositionAssignment

- Contains assignment problem solvers. Mukres, DHBA and Greedy