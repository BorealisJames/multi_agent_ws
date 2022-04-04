#pragma once

#include <chrono>
#include <functional>
#include <set>
#include <unordered_map>
#include <vector>

#include "Munkres.h"

namespace dhba
{

using DHBATaskAssignments = std::unordered_map<int, uint32_t>; //dhba assignment of task to agent (agent, task)

class DHBA
{

public:

	typedef std::unordered_map<uint32_t, double> cost_of_task;	//taskid, cost
	typedef std::unordered_map<int, cost_of_task> cost_of_agent;//agent id, cost_of_task

	DHBA();

	DHBA(const std::vector<uint32_t>& tasks_id,
			const int agent_id,
			const int64_t expiry_duration_usec,
			const std::function<bool
									(const uint32_t task_id,
									double& cost)>& cost_for_task_func);

	void Init(const std::vector<uint32_t>& tasks_id,
				const int agent_id,
				const int64_t expiry_duration_usec,
				const std::function<bool
										(const uint32_t task_id,
										double& cost)>& cost_for_task_func);

	void UpdateCostTableAndAgentLastSeenTime(const int sender_id,
		const cost_of_agent& cost_table,
		const std::unordered_map<int, int64_t>& agents_last_seen_time);

	void AssignTask();


	//get agent id
	int GetAgentID() const;

	//get task assigned to own agent
	bool GetOwnAssignedTask(uint32_t& tasks_id) const;

	//get tasks assigned to every agent
	void GetAssignedTaskMap(DHBATaskAssignments& task_assignment) const;

	//get own agent's cost table and time to be broadcasted
	void GetCostTableAndAgentsTime(cost_of_agent& cost_table,
									std::unordered_map<int, int64_t>& agents_last_seen_time) const;

	//get agents that are participating in the task
	void GetAgentsInTeam(std::vector<int>& participating_agents_vec) const;
	void GetPotentialListOfTaskObservableByTeam(std::vector<uint32_t>& participating_tasks_vec) const;

	//get task that agent will try to do
	void GetPotentialListOfTaskObservableByOwnAgent(std::vector<uint32_t>& potential_tasks_to_be_assigned) const;

	//add task for agent to do
	void AddTaskToPotentialList(const uint32_t task_id);

	//remove task that agent has to do
	void RemoveTaskFromPotentialList(const uint32_t task_id);

	//replace potential task list
	void ReplaceAndResetPotentialListOfTask(const std::vector<uint32_t>& tasks_id);

private:

	void GetOwnCostOfTasks(cost_of_task& own_cost_of_tasks) const;

	void UpdateCostTableAndAgentLastSeenTimeForOwnAgent();

	void GetListOfParticipatingAgentsAndTasks(std::vector<int>& participating_agents_vec, std::vector<uint32_t>& participating_tasks_vec) const;

	void PopulateMatrixWithCost(const std::vector<int>& participating_agents_vec, 
								const std::vector<uint32_t>& participating_tasks_vec,
								Matrix<double>& matrix) const;

	void UpdateAssignedTaskMap(const std::vector<int>& participating_agents_vec,
								const std::vector<uint32_t>& participating_tasks_vec,
								const Matrix<double>& matrix);

	int agent_id_;

	std::vector<uint32_t> potential_tasks_to_be_assigned_;

	int64_t expiry_duration_usec_;

	std::function<bool
						(const uint32_t task_id,
						double& cost)> cost_for_task_func_;

	cost_of_agent cost_table_;

	std::unordered_map<int, int64_t> agents_last_seen_time_;

	std::unordered_map<int, uint32_t> assigned_task_map_;

	std::vector<int> participating_agents_vec_;
	std::vector<uint32_t> participating_tasks_vec_;
};

}// namespace dhba