#include "lrf_global_planner/dijkstra_planner.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(lrf_global_planner::DijkstraPlanner, nav_core::BaseGlobalPlanner)

namespace lrf_global_planner {

DijkstraPlanner::DijkstraPlanner() : costmap_ros_(nullptr), costmap_(nullptr), initialized_(false) {}

DijkstraPlanner::DijkstraPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : DijkstraPlanner() {
    initialize(name, costmap_ros);
}

void DijkstraPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("allow_unknown", allow_unknown_, true);

        initialized_ = true;
        ROS_INFO("DijkstraPlanner initialized.");
    }
}

bool DijkstraPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    plan.clear();

    if (start.header.frame_id != costmap_ros_->getGlobalFrameID() ||
        goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
        ROS_ERROR("Frame ID mismatch: expected %s", costmap_ros_->getGlobalFrameID().c_str());
        return false;
    }

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_WARN("Start or goal out of bounds.");
        return false;
    }

    unsigned int map_size = width_ * height_;
    std::vector<float> cost_map(map_size, std::numeric_limits<float>::infinity());
    std::vector<std::pair<int, int>> parent_map(map_size, {-1, -1});
    using Cell = std::pair<float, std::pair<int, int>>;
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open_list;

    cost_map[toIndex(start_x, start_y)] = 0.0f;
    open_list.push({0.0, {start_x, start_y}});

    while (!open_list.empty()) {
        auto [current_cost, current_cell] = open_list.top();
        open_list.pop();
        auto [x, y] = current_cell;
        if (x == goal_x && y == goal_y) break;

        for (auto [nx, ny] : getNeighbors(x, y)) {
            if (!isValid(nx, ny)) continue;
            float cell_cost = getCost(nx, ny);
            if (cell_cost == std::numeric_limits<float>::infinity()) continue;

            float new_cost = current_cost + cell_cost;
            if (new_cost < cost_map[toIndex(nx, ny)]) {
                cost_map[toIndex(nx, ny)] = new_cost;
                parent_map[toIndex(nx, ny)] = {x, y};
                open_list.push({new_cost, {nx, ny}});
            }
        }
    }

    if (parent_map[toIndex(goal_x, goal_y)].first == -1) {
        ROS_WARN("No path to goal found.");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> raw_path;
    reconstructPath(start_x, start_y, goal_x, goal_y, parent_map, raw_path);

    for (const auto& pose : raw_path) {
        geometry_msgs::PoseStamped world_pose;
        double wx, wy;
        costmap_->mapToWorld(pose.pose.position.x, pose.pose.position.y, wx, wy);
        world_pose.pose.position.x = wx;
        world_pose.pose.position.y = wy;
        world_pose.pose.orientation.w = 1.0;
        world_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        world_pose.header.stamp = ros::Time::now();
        plan.push_back(world_pose);
    }

    return !plan.empty();
}

// ---------- Helper Functions ----------

int DijkstraPlanner::toIndex(int x, int y) const {
    return y * width_ + x;
}

bool DijkstraPlanner::isValid(int x, int y) const {
    return x >= 0 && x < static_cast<int>(width_) && y >= 0 && y < static_cast<int>(height_);
}

std::vector<std::pair<int, int>> DijkstraPlanner::getNeighbors(int x, int y) const {
    return {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
}

float DijkstraPlanner::getCost(int x, int y) const {
    unsigned char cost = costmap_->getCost(x, y);
    return (cost == costmap_2d::LETHAL_OBSTACLE || (!allow_unknown_ && cost == costmap_2d::NO_INFORMATION))
           ? std::numeric_limits<float>::infinity() : static_cast<float>(cost);
}

void DijkstraPlanner::reconstructPath(int start_x, int start_y, int goal_x, int goal_y,
                                      const std::vector<std::pair<int, int>>& parent_map,
                                      std::vector<geometry_msgs::PoseStamped>& plan) const {
    std::vector<std::pair<int, int>> path_cells;
    int x = goal_x, y = goal_y;

    while (!(x == start_x && y == start_y)) {
        path_cells.emplace_back(x, y);
        auto [px, py] = parent_map[toIndex(x, y)];
        if (px == -1 && py == -1) {
            ROS_WARN("Broken path.");
            plan.clear();
            return;
        }
        x = px;
        y = py;
    }
    path_cells.emplace_back(start_x, start_y);
    std::reverse(path_cells.begin(), path_cells.end());

    for (const auto& [px, py] : path_cells) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = px;
        pose.pose.position.y = py;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
}

}  // namespace lrf_global_planner
