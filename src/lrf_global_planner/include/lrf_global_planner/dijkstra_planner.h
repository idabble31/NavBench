#ifndef DIJKSTRA_PLANNER_H
#define DIJKSTRA_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <vector>
#include <utility>

namespace lrf_global_planner {

class DijkstraPlanner : public nav_core::BaseGlobalPlanner {
public:
    DijkstraPlanner();  // Empty constructor for pluginlib
    DijkstraPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);  // Optional constructor

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    // Helper functions (WITH const)
    int toIndex(int x, int y) const;
    bool isValid(int x, int y) const;
    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;
    float getCost(int x, int y) const;
    void reconstructPath(int start_x, int start_y, int goal_x, int goal_y, 
                         const std::vector<std::pair<int, int>>& parent_map,
                         std::vector<geometry_msgs::PoseStamped>& plan) const;

    // Member variables
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    unsigned int width_, height_;
    bool initialized_;
    bool allow_unknown_;
};

}  // namespace lrf_global_planner

#endif  // DIJKSTRA_PLANNER_H
