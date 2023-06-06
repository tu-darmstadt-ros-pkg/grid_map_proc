#include <grid_map_proc/grid_map_transforms.h>


#include <opencv2/highgui/highgui.hpp>

namespace grid_map_transforms{

  bool addInflatedLayer(grid_map::GridMap& grid_map,
                                     const float inflation_radius_map_cells,
                                     const std::string occupancy_layer,
                                     const std::string inflated_occupancy_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    cv::Mat map_mat = cv::Mat(grid_map.getSize()(0), grid_map.getSize()(1), CV_8UC1);

    uchar *input = (uchar*)(map_mat.data);

    size_t size_x = grid_map.getSize()(0);
    size_t size_y = grid_map.getSize()(1);

    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
      for (size_t idx_y = 0; idx_y < size_y; ++idx_y){
        input[map_mat.cols * idx_x + idx_y] = (grid_data(idx_x, idx_y)) == 100.0 ? 255 : 0 ;
      }
    }

    grid_map.add(inflated_occupancy_layer);
    grid_map::Matrix& data_inflated (grid_map[inflated_occupancy_layer]);

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = inflation_radius_map_cells;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );

    cv::Mat inflated_mat = cv::Mat(grid_map.getSize()(0), grid_map.getSize()(1), CV_8UC1);
    uchar *inflated_map_p = (uchar*)(inflated_mat.data);

    cv::dilate(map_mat, inflated_mat, element);

    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
      for (size_t idx_y = 0; idx_y < size_y; ++idx_y){

        if (grid_data (idx_x, idx_y) != 0.0){
          // If not free space, copy old map
          data_inflated(idx_x, idx_y) = grid_data(idx_x, idx_y);
        }else{

          if (inflated_map_p[map_mat.cols * idx_x + idx_y] != 0){
            // If free space and inflated in dilated map, mark occupied
            data_inflated(idx_x, idx_y) = 100.0;
          }else{
            // Otherwise copy old map
            data_inflated(idx_x, idx_y) = grid_data (idx_x, idx_y);
          }
        }
      }
    }

    return true;
  }

  bool addDeflatedLayer(grid_map::GridMap& grid_map,
                                     const float deflation_radius_map_cells,
                                     const std::string occupancy_layer,
                                     const std::string deflated_occupancy_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    cv::Mat map_mat = cv::Mat(grid_map.getSize()(0), grid_map.getSize()(1), CV_8UC1);

    uchar *input = (uchar*)(map_mat.data);

    size_t size_x = grid_map.getSize()(0);
    size_t size_y = grid_map.getSize()(1);

    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
      for (size_t idx_y = 0; idx_y < size_y; ++idx_y){
        input[map_mat.cols * idx_x + idx_y] = (grid_data(idx_x, idx_y)) == 100.0 ? 255 : 0 ;
      }
    }

    grid_map.add(deflated_occupancy_layer);
    grid_map::Matrix& data_deflated (grid_map[deflated_occupancy_layer]);

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = deflation_radius_map_cells;
    cv::Mat element = cv::getStructuringElement( erosion_type,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );

    cv::Mat deflated_mat = cv::Mat(grid_map.getSize()(0), grid_map.getSize()(1), CV_8UC1);
    uchar *deflated_map_p = (uchar*)(deflated_mat.data);

    cv::erode(map_mat, deflated_mat, element);

    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
      for (size_t idx_y = 0; idx_y < size_y; ++idx_y){

        if (grid_data (idx_x, idx_y) == 0.0){
          // If free space, copy old map
          data_deflated(idx_x, idx_y) = grid_data(idx_x, idx_y);
        }else{

          if (deflated_map_p[map_mat.cols * idx_x + idx_y] != 255){
            // If not free space and deflated in eroded map, mark free
            data_deflated(idx_x, idx_y) = 0.0;
          }else{
            // Otherwise copy old map
            data_deflated(idx_x, idx_y) = grid_data (idx_x, idx_y);
          }
        }
      }
    }

    return true;
  }
  
  bool addDistanceTransformCv(grid_map::GridMap& grid_map,
                              const std::string occupancy_layer,
                              const std::string dist_trans_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    cv::Mat map_mat = cv::Mat(grid_map.getSize()(0), grid_map.getSize()(1), CV_8UC1);
    
    
    float lowerValue = 100.0;
    float upperValue = 0.0;
    
    uchar *input = (uchar*)(map_mat.data);
    
    float inv_up_subtr_low = 1.0 / (upperValue - lowerValue);
    
    //grid_map::Index index;
    size_t size_x = grid_map.getSize()(0);
    size_t size_y = grid_map.getSize()(1);

    for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
      for (size_t idx_y = 0; idx_y < size_y; ++idx_y){
        //input[map_mat.cols * idx_x + idx_y] = (uchar) (((grid_data(idx_x, idx_y) - lowerValue) * inv_up_subtr_low) * (float) 255);
        input[map_mat.cols * idx_x + idx_y] = (grid_data(idx_x, idx_y)) == 100.0 ? 0 : 255 ;
      }
    }

    //cv::namedWindow("converted_map");
    //cv::imshow("converted_map", map_mat);
    //cv::waitKey();

    grid_map.add(dist_trans_layer);
    grid_map::Matrix& data_normal (grid_map[dist_trans_layer]);
    
    //Have to work around row vs column major representation in cv vs eigen
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data (data_normal);
    
    cv::Mat distance_transformed (data.rows(), data.cols(), CV_32FC1, data.data());
    
    // @TODO Appears OpenCV 2.4 broken in that it does not provide required enums. Looked up and manually added values
    // https://github.com/opencv/opencv/blob/master/modules/imgproc/include/opencv2/imgproc.hpp#L308
    cv::distanceTransform(map_mat, distance_transformed, 2, 3);
    
    data_normal = data;

    return true;
  }

  bool addDistanceTransform(grid_map::GridMap& grid_map,
                            const grid_map::Index& seed_point,
                            std::vector<grid_map::Index>& obstacle_cells,
                            std::vector<grid_map::Index>& frontier_cells,
                            float min_frontier_dist_,
                            int min_frontier_size,
                            const std::string occupancy_layer,
                            const std::string dist_trans_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    grid_map.add(dist_trans_layer, std::numeric_limits<float>::max());
    grid_map::Matrix& dist_layer (grid_map[dist_trans_layer]);

    std::queue<grid_map::Index> point_queue;

    obstacle_cells.clear();
    frontier_cells.clear();


    collectReachableObstacleCells(grid_map,
                                  seed_point,
                                  obstacle_cells,
                                  frontier_cells,
                                  min_frontier_dist_,
                                  min_frontier_size);

    for (size_t i = 0; i < obstacle_cells.size(); ++i){
      const grid_map::Index& point = obstacle_cells[i];
      dist_layer(point(0), point(1)) = 0.0;
      point_queue.push(point);
    }

    size_t size_x_lim = grid_map.getSize()(0) -1;
    size_t size_y_lim = grid_map.getSize()(1) -1;

    float adjacent_dist = 0.955;
    float diagonal_dist = 1.3693;

    while (point_queue.size()){
      grid_map::Index point (point_queue.front());
      point_queue.pop();

      //Reject points near border here early as to not require checks later
      if (point(0) < 1 || point(0) >= size_x_lim ||
          point(1) < 1 || point(1) >= size_y_lim){
          continue;
      }

      float current_val = dist_layer(point(0), point(1));

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)-1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0),
                      point(1)-1,
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)+1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)-1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)+1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)-1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0),
                      point(1)+1,
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchDistCell(grid_data,
                      dist_layer,
                      point(0)+1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      point_queue);
    }

    return true;

  }

  bool addExplorationTransform(grid_map::GridMap& grid_map,
                            const std::vector<grid_map::Index>& goal_points,
                            const float lethal_dist,
                            const float penalty_dist,
                            const float penalty_weight,
                            const std::string occupancy_layer,
                            const std::string dist_trans_layer,
                            const std::string expl_trans_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    if (!grid_map.exists(dist_trans_layer))
      return false;

    const grid_map::Matrix& grid_data (grid_map[occupancy_layer]);
    const grid_map::Matrix& dist_data (grid_map[dist_trans_layer]);

    grid_map.add(expl_trans_layer, std::numeric_limits<float>::max());
    grid_map::Matrix& expl_layer (grid_map[expl_trans_layer]);

    std::queue<grid_map::Index> point_queue;

    for (size_t i = 0; i < goal_points.size(); ++i){
      const grid_map::Index& point = goal_points[i];
      expl_layer(point(0), point(1)) = 0.0;
      point_queue.push(point);
    }

    size_t size_x_lim = grid_map.getSize()(0) -1;
    size_t size_y_lim = grid_map.getSize()(1) -1;

    float adjacent_dist = 0.955;
    float diagonal_dist = 1.3693;

    //std::cout << "pq size:" << point_queue.size() << "\n";

    while (point_queue.size()){
      grid_map::Index point (point_queue.front());
      point_queue.pop();

      //Reject points near border here early as to not require checks later
      if (point(0) < 1 || point(0) >= size_x_lim ||
          point(1) < 1 || point(1) >= size_y_lim){
          continue;
      }

      float current_val = expl_layer(point(0), point(1));

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)-1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0),
                      point(1)-1,
                      current_val,
                      adjacent_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)+1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)-1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)+1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)-1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0),
                      point(1)+1,
                      current_val,
                      adjacent_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);

      touchExplorationCell(grid_data,
                      dist_data,
                      expl_layer,
                      point(0)+1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      lethal_dist,
                      penalty_dist,
                      penalty_weight,
                      point_queue);
    }

    return true;
  }


  bool collectReachableObstacleCells(grid_map::GridMap& grid_map,
                                     const grid_map::Index& seed_point,
                                     std::vector<grid_map::Index>& obstacle_cells,
                                     std::vector<grid_map::Index>& frontier_cells,
                                     float min_frontier_dist,
                                     int min_frontier_size,
                                     const std::string occupancy_layer,
                                     const std::string dist_seed_layer)
  {

    if (!grid_map.exists(occupancy_layer))
      return false;

    auto resolution = static_cast<float>(grid_map.getResolution());

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    std::queue<grid_map::Index> point_queue;
    point_queue.push(seed_point);

    grid_map.add(dist_seed_layer, std::numeric_limits<float>::max());
    grid_map::Matrix& expl_layer (grid_map[dist_seed_layer]);

    expl_layer(seed_point(0), seed_point(1)) = 0.0;

    size_t size_x_lim = grid_map.getSize()(0) -1;
    size_t size_y_lim = grid_map.getSize()(1) -1;

    float min_distance_sq = std::pow(min_frontier_dist,2);

    while (point_queue.size()){

      grid_map::Index point (point_queue.front());
      point_queue.pop();

      //Reject points near border here early as to not require checks later
      if (point(0) < 1 || point(0) >= size_x_lim ||
          point(1) < 1 || point(1) >= size_y_lim){
          continue;
      }

      float current_val = expl_layer(point(0), point(1));

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)-1,
                           point(1)-1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0),
                           point(1)-1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)+1,
                           point(1)-1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)-1,
                           point(1),
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)+1,
                           point(1),
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)-1,
                           point(1)+1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0),
                           point(1)+1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           resolution,
                           seed_point,
                           point,
                           point(0)+1,
                           point(1)+1,
                           obstacle_cells,
                           frontier_cells,
                           point_queue,
                           min_distance_sq);
    }

      filterSmallFrontiersFullySurroundedByKnownCells(expl_layer, frontier_cells, min_frontier_size);
    return true;
  }

  void touchExplorationCell(const grid_map::Matrix& grid_map,
                            const grid_map::Matrix& dist_map,
                       grid_map::Matrix& expl_trans_map,
                       const int idx_x,
                       const int idx_y,
                       const float curr_val,
                       const float add_cost,
                       const float lethal_dist,
                       const float penalty_dist,
                       const float penalty_weight,
                       std::queue<grid_map::Index>& point_queue)
  {
    //If not free at cell, return right away
    if (grid_map(idx_x, idx_y) != 0)
      return;


    float dist = dist_map(idx_x, idx_y);

    if (dist < lethal_dist)
      return;

    float cost = curr_val + add_cost;

    //if (dist < 20.0){
    //  cost += 1.0 * std::pow((20.0 - dist_map(idx_x, idx_y)), 2);
    //}
    if (dist < penalty_dist){
      float add_cost = (penalty_dist - dist);
      cost += penalty_weight * (add_cost * add_cost);
    }

    if (expl_trans_map(idx_x, idx_y) > cost){
      expl_trans_map(idx_x, idx_y) = cost;
      point_queue.push(grid_map::Index(idx_x, idx_y));
    }
  }

  void touchDistCell(const grid_map::Matrix& grid_map,
                       grid_map::Matrix& expl_trans_map,
                       const int idx_x,
                       const int idx_y,
                       const float curr_val,
                       const float add_cost,
                       std::queue<grid_map::Index>& point_queue)
  {
    //If not free at cell, return right away
    if (grid_map(idx_x, idx_y) != 0)
      return;

    float cost = curr_val + add_cost;

    if (expl_trans_map(idx_x, idx_y) > cost){
      expl_trans_map(idx_x, idx_y) = cost;
      point_queue.push(grid_map::Index(idx_x, idx_y));
    }
  }

  void touchObstacleSearchCell(const grid_map::Matrix& grid_map,
                       grid_map::Matrix& expl_trans_map,
                       float resolution,
                       const grid_map::Index& start_point,
                       const grid_map::Index& current_point,
                       const int idx_x,
                       const int idx_y,
                       std::vector<grid_map::Index>& obstacle_cells,
                       std::vector<grid_map::Index>& frontier_cells,
                       std::queue<grid_map::Index>& point_queue,
                       float min_distance_sq)
  {
    // Free
    if ( (grid_map(idx_x, idx_y) == 0.0) ){
      if (expl_trans_map(idx_x, idx_y) != std::numeric_limits<float>::max()){
        return;
      }else{
        expl_trans_map(idx_x, idx_y) = -3.0;
        point_queue.push(grid_map::Index(idx_x, idx_y));
      }
    // Occupied
    }else if (grid_map(idx_x, idx_y) == 100.0){
      if (expl_trans_map(idx_x, idx_y) == -1.0){
        return;
      }else{
        expl_trans_map(idx_x, idx_y) = -1.0;
        obstacle_cells.push_back(grid_map::Index(idx_x, idx_y));
      }
    // Unknown
    }else{
      if (expl_trans_map(current_point(0), current_point(1)) == -2.0 || expl_trans_map(current_point(0), current_point(1)) == -4.0 ){
        return;
      }else{
        expl_trans_map(current_point(0), current_point(1)) = -2.0;
        // add to frontier if far enough away from robot
        float x_diff = static_cast<float>(current_point(0) - start_point(0)) * resolution;
        float y_diff = static_cast<float>(current_point(1) - start_point(1)) * resolution;
        if ( std::abs(x_diff * x_diff + y_diff * y_diff) > min_distance_sq){
            expl_trans_map(current_point(0), current_point(1)) = -4.0;
            frontier_cells.push_back(grid_map::Index(current_point(0), current_point(1)));
        }
      }
    }
  }

  void filterSmallFrontiersFullySurroundedByKnownCells(grid_map::Matrix& expl_trans_map,
                                                       std::vector<grid_map::Index>& frontier_cells,
                                                       int min_frontier_size)
  {
      if(frontier_cells.size()<min_frontier_size)
          return;
      if(min_frontier_size <1)
          return;
      std::vector<grid_map::Index> new_frontier_cells;
      //iterate over frontiers_cells all frontier cells and count cells per cluster
      std::queue<grid_map::Index> point_queue;
      std::vector<grid_map::Index> frontier_cell_cluster;
      for(auto& frontier_cell:frontier_cells)
      {
          if(expl_trans_map(frontier_cell(0), frontier_cell(1))==-12.0)
                  continue;
          frontier_cell_cluster.clear();
          point_queue.push(frontier_cell);
          touchFilterCell(expl_trans_map,
                          frontier_cell,
                          point_queue,
                          frontier_cell_cluster);
          while (!point_queue.empty())
          {
              grid_map::Index point = point_queue.front();
              point_queue.pop();
              //count connected frontier cells and check that non-frontier neighbors are not unknown
              for(int i=-1;i<=1;i++)
              {
                  for(int j=-1;j<=1;j++)
                  {
                      if(i==0 && j==0) continue;
                      touchFilterCell(expl_trans_map,
                                      {point[0]+i,point[1]+j},
                                      point_queue,
                                      frontier_cell_cluster);
                  }
              }
          }
          if(frontier_cell_cluster.size() > min_frontier_size)
          {
              for(const auto& cell:frontier_cell_cluster){
                  new_frontier_cells.push_back(cell);
              }
          }
      }
      if(new_frontier_cells.empty())
      {
          ROS_WARN_STREAM("After filtering small frontiers, no frontiers left -> using unfiltered frontiers");
          return;
      }
      frontier_cells = new_frontier_cells;
  }

  void touchFilterCell(grid_map::Matrix& expl_trans_map,
                       const grid_map::Index& index,
                       std::queue<grid_map::Index>& point_queue,
                       std::vector<grid_map::Index>& frontier_cell_cluster){
      if(expl_trans_map(index(0), index(1)) == -4.0) {
          point_queue.push(index);
          frontier_cell_cluster.emplace_back(index);
          //mark cell so that it is not found again
          // (if expl_trans_map is needed again, undo after
          // filterSmallFrontiersFullySurroundedByKnownCells for all frontier cells)
          expl_trans_map(index(0), index(1)) = -12.0;
      }
  }
} /* namespace */
