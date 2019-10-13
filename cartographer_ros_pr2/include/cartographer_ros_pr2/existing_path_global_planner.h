#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tuple>

namespace navfn
{
  class ExistingPathGlobalPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    ExistingPathGlobalPlanner();
    ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
    void existPathCallback(const visualization_msgs::MarkerArrayConstPtr &msg);

    ~ExistingPathGlobalPlanner(){}

    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

  protected:

    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    costmap_2d::Costmap2D* costmap_;
    ros::Publisher plan_pub_;
    ros::Subscriber existing_path_sub_;
    bool whole_linear_interpolation_;
    bool initialized_;
    std::vector<geometry_msgs::Point> existing_path_;

  private:

    void mapToWorld(double mx, double my, double& wx, double& wy);
    std::vector<geometry_msgs::PoseStamped> linearInterpolation(geometry_msgs::Point start_p, geometry_msgs::Point goal_p);
    std::tuple<int, geometry_msgs::Point > findClosestPoint(geometry_msgs::Point p, const std::vector<geometry_msgs::Point>& path);
    double default_tolerance_, path_resolution_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;
    std::string global_frame_;
  };
};

