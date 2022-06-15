
// Listen to HRI mode 
// Decouple the incoming msgs
// Record the human poses into history of human poses
// If activated + follow me, then pass the history of human poses into the formation planner
// If activated + go there, dont pass the history of human poses. Instead take the "Go there pose" pass it to consensus_path_planner.
// consensus_path_planner will generate a series of poses. Pass this poses to the formation planner instead.

// Params?