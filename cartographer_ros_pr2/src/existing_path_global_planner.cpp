#include <cartographer_ros_pr2/existing_path_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::ExistingPathGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace navfn {

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner()
    : costmap_(NULL), initialized_(false) {}

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL), initialized_(false) {

      initialize(name, costmap_ros);
  }

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL), initialized_(false) {

      initialize(name, costmap, global_frame);
  }

  void ExistingPathGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
      global_frame_ = global_frame;

      existing_path_.resize(0);

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      std::string topic_name;
      private_nh.param("existing_path_topic_name", topic_name, std::string("trajectory_list"));
      existing_path_sub_ = private_nh.subscribe(topic_name, 1, &ExistingPathGlobalPlanner::existPathCallback, this);

      private_nh.param("default_tolerance", default_tolerance_, 0.0);
      private_nh.param("whole_linear_interpolation", whole_linear_interpolation_, false);
      private_nh.param("path_resolution", path_resolution_, 0.025);


      make_plan_srv_ =  private_nh.advertiseService("make_plan", &ExistingPathGlobalPlanner::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void ExistingPathGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }


  bool ExistingPathGlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  }

  void ExistingPathGlobalPlanner::existPathCallback(const visualization_msgs::MarkerArrayConstPtr &msg)
  {
    geometry_msgs::Pose pose;
    for(auto const seg: msg->markers)
      {
        /* only extract the fixed trajectory */
        if(seg.ns == std::string("Trajectory 0"))
          {

            for(auto const point: seg.points)
              {
                if(existing_path_.size() == 0)
                  {
                    existing_path_.push_back(point);
                    ROS_INFO("Add [%f, %f] to existing path", point.x, point.y);
                  }
                else
                  {
                    double dist = std::sqrt(std::pow(existing_path_.back().y - point.y, 2) + std::pow(existing_path_.back().y - point.y, 2));
                    if(dist > path_resolution_)
                      {
                        existing_path_.push_back(point);
                        ROS_INFO("Add [%f, %f] to existing path", point.x, point.y);
                      }
                  }
              }
          }
      }
    existing_path_sub_.shutdown();
  }

  void ExistingPathGlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  void ExistingPathGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool ExistingPathGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  bool ExistingPathGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    /*
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, mx, my);
    */

    /*
    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
    */

    int map_start[2];
    /*
    map_start[0] = mx;
    map_start[1] = my;
    */
    map_start[0] = wx; // in world frame
    map_start[1] = wy;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    /*
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      if(tolerance <= 0.0){
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }
    */

    int map_goal[2];
    map_goal[0] = wx;
    map_goal[1] = wy;

    // linear interpolation
    ros::Time plan_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    auto linearInterpolation = [this, &pose](geometry_msgs::Point start_p, geometry_msgs::Point goal_p)
      {
        std::vector<geometry_msgs::PoseStamped> linear_plan;

        double path_len = std::sqrt(std::pow(goal_p.x - start_p.x, 2) + std::pow(goal_p.y - start_p.y, 2));
        int path_num = path_len / path_resolution_;

        for(int i = 0; i <= path_num; i++)
          {
            pose.pose.position.x = (goal_p.x - start_p.x) * i / path_num + start_p.x;
            pose.pose.position.y = (goal_p.y - start_p.y) * i / path_num + start_p.y;
            linear_plan.push_back(pose);
          }

        pose.pose.position = goal_p;
        linear_plan.push_back(pose);

        return linear_plan;
      };

    if(whole_linear_interpolation_)
      {
        plan = linearInterpolation(start.pose.position, goal.pose.position);
      }
    else
      {
        plan = linearInterpolation(start.pose.position, existing_path_.front());

        for(auto const & point: existing_path_)
          {
            pose.pose.position = point;
            plan.push_back(pose);
          }

        /* TODO: the end interpolation is not reasonable */
        auto plan_tmp = linearInterpolation(existing_path_.back(), goal.pose.position);
        plan.insert(plan.end(), plan_tmp.begin() + 1, plan_tmp.end());
      }

    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    return !plan.empty();
  }

  void ExistingPathGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }
};
