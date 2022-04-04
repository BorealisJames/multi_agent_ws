#include "../include/DHBA.h"

namespace dhba
{

DHBA::DHBA()
	: agent_id_(0)
	, expiry_duration_usec_(-1)
{
}

DHBA::DHBA(const std::vector<uint32_t>& tasks_id,
			const int agent_id,
			const int64_t expiry_duration_usec,
			const std::function<bool
									(const uint32_t task_id,
									 double& cost)>& cost_for_task_func)
	: potential_tasks_to_be_assigned_(tasks_id)
	, agent_id_(agent_id)
	, expiry_duration_usec_(expiry_duration_usec)
	, cost_for_task_func_(cost_for_task_func)
{
}

void 
DHBA::Init(const std::vector<uint32_t>& tasks_id,
			const int agent_id,
			const int64_t expiry_duration_usec,
			const std::function<bool
									(const uint32_t task_id,
									 double& cost)>& cost_for_task_func)
{
	potential_tasks_to_be_assigned_ = tasks_id;
	agent_id_ = agent_id;
	expiry_duration_usec_ = expiry_duration_usec;
	cost_for_task_func_ = cost_for_task_func;
}

void 
DHBA::UpdateCostTableAndAgentLastSeenTime(const int sender_id,
											const cost_of_agent& others_cost_table,
											const std::unordered_map<int, int64_t>& others_agents_last_seen_time)
{
	if (sender_id == agent_id_)
	{
		return;
	}

	for (auto&& others_cost_info : others_cost_table)
	{
		int agent_of_interest = others_cost_info.first;

		//use last seen time to decide if should add or reject
		auto own_agents_last_seen_time_itr = agents_last_seen_time_.find(agent_of_interest);
		if (own_agents_last_seen_time_itr == agents_last_seen_time_.end())
		{
			agents_last_seen_time_[agent_of_interest] = others_agents_last_seen_time.find(agent_of_interest)->second;
			cost_table_[agent_of_interest] = others_cost_table.find(agent_of_interest)->second;
		}
		else if (own_agents_last_seen_time_itr->second < others_agents_last_seen_time.find(agent_of_interest)->second)
		{
			agents_last_seen_time_[agent_of_interest] = others_agents_last_seen_time.find(agent_of_interest)->second;
			cost_table_[agent_of_interest] = others_cost_table.find(agent_of_interest)->second;
		}
	}
}

void 
DHBA::AssignTask()
{
	UpdateCostTableAndAgentLastSeenTimeForOwnAgent();

	GetListOfParticipatingAgentsAndTasks(participating_agents_vec_, participating_tasks_vec_);

	//create matrix, agents are rows and tasks are columns
	int nrows = participating_agents_vec_.size();
	int ncols = participating_tasks_vec_.size();

	assigned_task_map_.clear();

	if (nrows == 0 || ncols == 0)
	{
		return;
	}

	Matrix<double> matrix(nrows, ncols);
	PopulateMatrixWithCost(participating_agents_vec_,
		participating_tasks_vec_,
		matrix);

	// Apply Munkres algorithm to matrix.
	Munkres<double> m;
	m.solve(matrix);

	// Update task assignments
	UpdateAssignedTaskMap(participating_agents_vec_,
						  participating_tasks_vec_,
						  matrix);
}

int
DHBA::GetAgentID() const
{
	return agent_id_;
}

bool 
DHBA::GetOwnAssignedTask(uint32_t& tasks_id) const
{
	bool retVal = false;

	auto assigned_task_map_itr = assigned_task_map_.find(agent_id_);
	if (assigned_task_map_itr != assigned_task_map_.end())
	{
		retVal = true;
		tasks_id = assigned_task_map_itr->second;
	}

	return retVal;
}

void 
DHBA::GetAssignedTaskMap(DHBATaskAssignments& task_assignment) const
{
	task_assignment = assigned_task_map_;
}

void 
DHBA::GetCostTableAndAgentsTime(cost_of_agent& cost_table,
								std::unordered_map<int, int64_t>& agents_last_seen_time) const
{
	cost_table = cost_table_;
	agents_last_seen_time = agents_last_seen_time_;
}

void 
DHBA::GetAgentsInTeam(std::vector<int>& participating_agents_vec) const
{
	participating_agents_vec = participating_agents_vec_;
}

void 
DHBA::GetPotentialListOfTaskObservableByTeam(std::vector<uint32_t>& participating_tasks_vec) const
{
	participating_tasks_vec = participating_tasks_vec_;
}

void 
DHBA::GetPotentialListOfTaskObservableByOwnAgent(std::vector<uint32_t>& potential_tasks_to_be_assigned) const
{
	potential_tasks_to_be_assigned = potential_tasks_to_be_assigned_;
}

void 
DHBA::AddTaskToPotentialList(const uint32_t task_id)
{
	if (std::find(potential_tasks_to_be_assigned_.begin(), 
					potential_tasks_to_be_assigned_.end(), 
					task_id) == 
		potential_tasks_to_be_assigned_.end())
	{
		potential_tasks_to_be_assigned_.push_back(task_id);
	}
}

void 
DHBA::RemoveTaskFromPotentialList(const uint32_t task_id)
{
	auto itr = std::find(potential_tasks_to_be_assigned_.begin(),
						potential_tasks_to_be_assigned_.end(),
						task_id);

	if (itr != potential_tasks_to_be_assigned_.end())
	{
		potential_tasks_to_be_assigned_.erase(itr);
	}
}

void 
DHBA::ReplaceAndResetPotentialListOfTask(const std::vector<uint32_t>& tasks_id)
{
	potential_tasks_to_be_assigned_.clear();
	potential_tasks_to_be_assigned_ = tasks_id;
}

void 
DHBA::GetOwnCostOfTasks(cost_of_task& own_cost_of_tasks) const
{
	own_cost_of_tasks.clear();

	for (auto&& task_id : potential_tasks_to_be_assigned_)
	{
		double cost;
		if (!cost_for_task_func_(task_id, cost))
		{
			cost = std::numeric_limits<double>::infinity();
		}
		own_cost_of_tasks[task_id] = cost;
	}
}

void 
DHBA::UpdateCostTableAndAgentLastSeenTimeForOwnAgent()
{
	//get own task cost
	cost_of_task own_cost_of_tasks;
	GetOwnCostOfTasks(own_cost_of_tasks);

	auto time_now = std::chrono::system_clock::now();
	auto time_now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch()).count();

	//update cost table and agent last seen time
	agents_last_seen_time_[agent_id_] = time_now_in_microseconds;
	cost_table_[agent_id_] = own_cost_of_tasks;
}

void 
DHBA::GetListOfParticipatingAgentsAndTasks(std::vector<int>& participating_agents_vec, std::vector<uint32_t>& participating_tasks_vec) const
{
	participating_agents_vec.clear();
	participating_tasks_vec.clear();

	auto time_now = std::chrono::system_clock::now();
	auto time_now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch()).count();

	//determine number of agents that are not expired
	for (auto&& agent_time : agents_last_seen_time_)
	{
		int agent_id = agent_time.first;
		int64_t last_seen_time = agent_time.second;

		if (agent_id == agent_id_)
		{
			participating_agents_vec.push_back(agent_id);
		}
		else if (time_now_in_microseconds - last_seen_time <= expiry_duration_usec_ || expiry_duration_usec_ <= 0)
		{
			participating_agents_vec.push_back(agent_id);
		}
	}

	//determine the union of tasks from unexpired agents
	std::set<uint32_t> set_of_participating_tasks;
	for (auto&& agent_not_expired : participating_agents_vec)
	{
		auto cost_table_itr = cost_table_.find(agent_not_expired);
		if (cost_table_itr != cost_table_.end())
		{
			for (auto&& task_and_cost : cost_table_itr->second)
			{
				uint32_t task_id = task_and_cost.first;
				double cost = task_and_cost.second;
				if (!isinf(cost))	//there should be no cost with inf value
				{
					set_of_participating_tasks.insert(task_id);
				}
			}
		}
	}
	participating_tasks_vec.assign(set_of_participating_tasks.begin(), set_of_participating_tasks.end());
}

void 
DHBA::PopulateMatrixWithCost(const std::vector<int>& participating_agents_vec,
							const std::vector<uint32_t>& participating_tasks_vec,
							Matrix<double>& matrix) const
{
	int nrows = participating_agents_vec.size();
	int ncols = participating_tasks_vec.size();

	for (int row = 0; row < nrows; row++)
	{
		for (int col = 0; col < ncols; col++)
		{

			double cost = std::numeric_limits<double>::infinity();

			int agent_of_interest = participating_agents_vec.at(row);
			uint32_t task_of_interest = participating_tasks_vec.at(col);

			auto cost_table_itr = cost_table_.find(agent_of_interest);
			if (cost_table_itr != cost_table_.end())
			{
				auto cost_of_task_itr = cost_table_itr->second.find(task_of_interest);
				if (cost_of_task_itr != cost_table_itr->second.end())
				{
					cost = cost_of_task_itr->second;
				}
			}

			matrix(row, col) = cost;

		}
	}
}

void 
DHBA::UpdateAssignedTaskMap(const std::vector<int>& participating_agents_vec,
							const std::vector<uint32_t>& participating_tasks_vec,
							const Matrix<double>& matrix)
{
	int nrows = participating_agents_vec.size();
	int ncols = participating_tasks_vec.size();

	for (int row = 0; row < nrows; row++)
	{
		for (int col = 0; col < ncols; col++)
		{
			if (matrix(row, col) == 0)
			{
				int agent_of_interest = participating_agents_vec.at(row);
				uint32_t task_of_interest = participating_tasks_vec.at(col);

				assigned_task_map_[agent_of_interest] = task_of_interest;
			}
		}
	}
}

}// namespace dhba