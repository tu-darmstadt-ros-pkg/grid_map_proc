#include <grid_map_proc/grid_map_path_planning.h>


#include <opencv2/highgui/highgui.hpp>

namespace grid_map_path_planning{
  
  
  bool findPathExplorationTransform(grid_map::GridMap& grid_map,
                                    const geometry_msgs::Pose& start_pose,
                                    std::vector<geometry_msgs::PoseStamped>& path,
                                    float* path_cost,
                                    const std::string occupancy_layer,
                                    const std::string dist_trans_layer,
                                    const std::string expl_trans_layer)
  {

    grid_map::Matrix& expl_data = grid_map[expl_trans_layer];

    grid_map::Index current_index;
    grid_map::Index next_index;

    if (!grid_map.getIndex(grid_map::Position(start_pose.position.x, start_pose.position.y),current_index)){
      ROS_WARN("Start index not in map");
      return false;
    }

    if (expl_data(current_index(0), current_index(1)) == std::numeric_limits<float>::max()){
      ROS_WARN("Start index not in exploration transform");
      return false;
    }

    std::vector <grid_map::Index> path_indices;
    path_indices.push_back(current_index);

    // Cost can be looked up from start pose
    //float path_cost_sum;


    //geometry_msgs::PoseStamped trajPoint;
    //std::string global_frame = costmap_ros_->getGlobalFrameID();
    //trajPoint.header.frame_id = global_frame;

    //std::cout << "\nStart curr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";

    grid_map::Matrix& dist_data = grid_map[dist_trans_layer];

    if (path_cost){
      *path_cost = expl_data(current_index(0), current_index(1));
    }


    while(!expl_data(current_index(0), current_index(1)) == 0.0)
    {

      // We guarantee in construction of expl. transform that we're not
      // at the border.
      //if (point(0) < 1 || point(0) >= size_x_lim ||
      //    point(1) < 1 || point(1) >= size_y_lim){
      //    continue;
      //}

      //std::cout << "\nStartloop curr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";



      float lowest_cost = std::numeric_limits<float>::min();


      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1),
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0),
                             current_index(1)-1,
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0),
                             current_index(1)+1,
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1),
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1)-1,
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1)+1,
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1)-1,
                             lowest_cost,
                             next_index);

      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1)+1,
                             lowest_cost,
                             next_index);


      //std::cout << "\ncurr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";

      if (lowest_cost == std::numeric_limits<float>::min()){
        ROS_WARN("Cannot find gradient");
        return false;
      }

      current_index = next_index;

      //if ((dist_data(current_index(0), current_index(1)) < 12.0) || expl_data(current_index(0), current_index(1)) == 0.0){
      //  path_indices.push_back(current_index);
      //  std::cout << "add: " << dist_data(current_index(0), current_index(1)) << "\n";
      //}else{
      //  std::cout << "rem: " << dist_data(current_index(0), current_index(1)) << "\n";
      //}

      path_indices.push_back(current_index);
    }

    //std::cout << "current_index after path gen: " << current_index << "\n";

    if (path_indices.size() == 1)
    {
      //Already at start
    }

    std::vector <grid_map::Index> refined_path_indices;

    shortCutPath(grid_map, path_indices, refined_path_indices);

    path_indices = refined_path_indices;

    //path.header.frame_id = "map";
    //path.header.stamp = ros::Time::now();
    path.resize(path_indices.size());

    for (size_t i = 0; i < path_indices.size(); ++i){
      grid_map::Position position;
      grid_map.getPosition(path_indices[i], position);

      geometry_msgs::Pose& pose = path[i].pose;

      pose.position.x = position(0);
      pose.position.y = position(1);

      if (i < (path_indices.size()-1)){
        float yaw = std::atan2(path_indices[i](1)-path_indices[i+1](1),
                               path_indices[i](0)-path_indices[i+1](0));

        pose.orientation.z = sin(yaw*0.5f);
        pose.orientation.w = cos(yaw*0.5f);
      }else{
        const geometry_msgs::Pose& prior_pose = path[i-1].pose;

        pose.orientation = prior_pose.orientation;
      }

    }

    return true;
  }

  bool findValidClosePoseExplorationTransform(grid_map::GridMap& grid_map,
                          const grid_map::Index& start_index,
                          grid_map::Index& adjusted_index,
                          const float allowed_distance_from_start,
                          const float required_final_distance,
                          const float desired_final_distance,
                          const std::string occupancy_layer,
                          const std::string dist_trans_layer,
                          const std::string expl_trans_layer)
  {
    //grid_map::Matrix& expl_data = grid_map[expl_trans_layer];

    grid_map::Index current_index;
    grid_map::Index next_index;

    //if (!grid_map.getIndex(grid_map::Position(start_index[0], start_index[1]),current_index)){
    //  ROS_WARN("Start index not in map");
    //  return false;
    //}

    current_index = start_index;

    //if (expl_data(current_index(0), current_index(1)) == std::numeric_limits<float>::max()){
    //  ROS_WARN("Start index not in exploration transform");
    //  return false;
    //}

    std::vector <grid_map::Index> path_indices;
    path_indices.push_back(current_index);

    // Cost can be looked up from start pose
    //float path_cost_sum;


    //geometry_msgs::PoseStamped trajPoint;
    //std::string global_frame = costmap_ros_->getGlobalFrameID();
    //trajPoint.header.frame_id = global_frame;

    //std::cout << "\nStart curr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";

    grid_map::Matrix& dist_data = grid_map[dist_trans_layer];

    float dist_from_start = 0.0f;


    float dist_from_obstacle = dist_data(current_index(0), current_index(1));

    if (dist_from_obstacle >= desired_final_distance){
      ROS_INFO("Pose already in free space, not modifying");
      adjusted_index = start_index;
      return true;
    }


    bool abort = false;

    while(!abort)
    {

      // We guarantee in construction of expl. transform that we're not
      // at the border.
      //if (point(0) < 1 || point(0) >= size_x_lim ||
      //    point(1) < 1 || point(1) >= size_y_lim){
      //    continue;
      //}

      //std::cout << "\nStartloop curr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";



      float highest_cost = 0.0f;


      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1),
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0),
                             current_index(1)-1,
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0),
                             current_index(1)+1,
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1),
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1)-1,
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1)+1,
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1)-1,
                             highest_cost,
                             next_index);

      touchDistanceField(dist_data,
                             current_index,
                             current_index(0)+1,
                             current_index(1)+1,
                             highest_cost,
                             next_index);


      dist_from_obstacle = dist_data(current_index(0), current_index(1));

      std::cout << "curr_index: " << current_index << "\ndist_from_obstacle: " << dist_from_obstacle << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

      // If gradient could not be followed..
      if (highest_cost == 0.0f){

        std::cout << "curr_index: " << current_index << "\nval: " << dist_data(current_index(0), current_index(1)) << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

        //ROS_INFO_STREAM("Reached end of distance transform with dist_from_start)

        // Could not reach enough clearance
        if (dist_from_obstacle < required_final_distance){
          ROS_WARN("Could not find gradient of distance transform leading to free area, returning original pose");
          adjusted_index = start_index;
          return false;

        // Could not reach desired distance from obstacles, but enough clearance
        }else if (dist_from_obstacle < desired_final_distance){
          ROS_WARN("Could not find gradient of distance transform reaching desired final distance");
          abort = true;

        }else{
          ROS_INFO("Reached final distance");
          abort = true;
        }

      // Gradient following worked
      }else{

        //dist_from_start += highest_cost;

        // If desired distance exceeded, stop here
        if (dist_from_obstacle > desired_final_distance){
          abort = true;

        // Otherwise continue gradient following
        }else{

          current_index = next_index;
          dist_from_start += highest_cost;
          path_indices.push_back(current_index);
        }
      }
    }

    adjusted_index = current_index;


    return true;
  }

  bool shortCutPath(grid_map::GridMap& grid_map,
                    const std::vector <grid_map::Index>& path_in,
                    std::vector <grid_map::Index>& path_out,
                    const std::string dist_trans_layer,
                    const std::string expl_trans_layer)
  {
    if (path_in.size() < 2){
      path_out = path_in;
      return true;
    }

    grid_map::Matrix& dist_data = grid_map[dist_trans_layer];
    grid_map::Matrix& expl_data = grid_map[expl_trans_layer];

    path_out.reserve(path_in.size());
    path_out.push_back(path_in[0]);

    size_t idx = 0;

    while (idx < path_in.size()-2){
      //std::cout << "idx: " << idx << " size: " << path_in.size() <<  "\n";
      const grid_map::Index& current_index (path_in[idx]);

      //if (idx >= path_in.size()-30)
      //  break;
      //if ((idx + 2) > (path_in.size() -10))
      //  break;

      for (size_t test_idx = idx + 2; test_idx < path_in.size(); ++test_idx){
        const grid_map::Index& test_index = path_in[test_idx];

        //std::cout << "idx: " << idx << " test_idx: " << test_idx << " size: " << path_in.size() << " \n";
        if (!shortCutValid(grid_map, dist_data, current_index, test_index)){
          idx = test_idx-1;
          break;
        }else{
          if (test_idx == (path_in.size()-1)){
            idx = test_idx;
            break;
          }
        }
      }

      //std::cout << "size: " << path_in.size() << " idx: " << idx << "\n";

      path_out.push_back(path_in[idx]);
    }

    if (idx < path_in.size()){
      path_out.push_back(path_in.back());
    }

    return true;
  }
  
} /* namespace */
