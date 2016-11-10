#include <grid_map_proc/grid_map_path_planning.h>


#include <opencv2/highgui/highgui.hpp>

namespace grid_map_path_planning{
  
  
  bool findPathExplorationTransform(grid_map::GridMap& grid_map,
                                    const geometry_msgs::Pose& start_pose,
                                    std::vector<geometry_msgs::PoseStamped>& path,
                                    std::string occupancy_layer,
                                    std::string expl_trans_layer)
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
                             current_index(1)-1,
                             lowest_cost,
                             next_index);


      touchGradientCell(expl_data,
                             current_index,
                             current_index(0)-1,
                             current_index(1),
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
                             current_index(1)-1,
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
                             current_index(0)+1,
                             current_index(1)+1,
                             lowest_cost,
                             next_index);


      //std::cout << "\ncurr_index:\n" << current_index << "\nval: " << expl_data(current_index(0), current_index(1)) << "\n";

      if (lowest_cost == std::numeric_limits<float>::min()){
        ROS_WARN("Cannot find gradient");
        return false;
      }

      //std::cout << "enter dat\n";
      //int bla;
      //std::cin >> bla;

      current_index = next_index;
      path_indices.push_back(current_index);
    }

    //path.header.frame_id = "map";
    //path.header.stamp = ros::Time::now();
    path.resize(path_indices.size());

    for (size_t i = 0; i < path_indices.size(); ++i){
      grid_map::Position position;
      grid_map.getPosition(path_indices[i], position);

      geometry_msgs::Pose& pose = path[i].pose;

      pose.position.x = position(0);
      pose.position.y = position(1);
      pose.orientation.w = 1.0;

    }

    return true;
  }
  
} /* namespace */
