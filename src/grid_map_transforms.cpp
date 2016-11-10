#include <grid_map_proc/grid_map_transforms.h>


#include <opencv2/highgui/highgui.hpp>

namespace grid_map_transforms{
  
  
  bool addDistanceTransformCv(grid_map::GridMap& grid_map,
                              std::string occupancy_layer,
                              std::string dist_trans_layer)
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
    
    //Have to work around row vs column major represnetation in cv vs eigen
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data (data_normal);
    
    cv::Mat distance_transformed (data.rows(), data.cols(), CV_32FC1, data.data());
    
    // @TODO Appears OpenCV 2.4 broken in that it does not provide required enums. Looked up and manually added values
    // https://github.com/opencv/opencv/blob/master/modules/imgproc/include/opencv2/imgproc.hpp#L308
    cv::distanceTransform(map_mat, distance_transformed, 2, 3);
    
    data_normal = data;

    return true;
  }

  bool addExplorationTransform(grid_map::GridMap& grid_map,
                            const std::vector<grid_map::Index>& goal_points,
                            std::string occupancy_layer,
                            std::string dist_trans_layer,
                            std::string expl_trans_layer)
  {
    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

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
                      expl_layer,
                      point(0)-1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0),
                      point(1)-1,
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0)+1,
                      point(1)-1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0)-1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0)+1,
                      point(1),
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0)-1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0),
                      point(1)+1,
                      current_val,
                      adjacent_dist,
                      point_queue);

      touchExplorationCell(grid_data,
                      expl_layer,
                      point(0)+1,
                      point(1)+1,
                      current_val,
                      diagonal_dist,
                      point_queue);
    }

    return true;
  }


  bool collectReachableObstacleCells(grid_map::GridMap& grid_map,
                                     const grid_map::Index& seed_point,
                                     std::vector<grid_map::Index>& obstacle_cells,
                                     std::vector<grid_map::Index>& frontier_cells,
                                     std::string occupancy_layer,
                                     std::string dist_seed_layer)
  {

    if (!grid_map.exists(occupancy_layer))
      return false;

    grid_map::Matrix& grid_data = grid_map[occupancy_layer];

    std::queue<grid_map::Index> point_queue;
    point_queue.push(seed_point);

    grid_map.add(dist_seed_layer, std::numeric_limits<float>::max());
    grid_map::Matrix& expl_layer (grid_map[dist_seed_layer]);

    expl_layer(seed_point(0), seed_point(1)) = 0.0;

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

      float current_val = expl_layer(point(0), point(1));

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)-1,
                           point(1)-1,
                           current_val,
                           diagonal_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0),
                           point(1)-1,
                           current_val,
                           adjacent_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)+1,
                           point(1)-1,
                           current_val,
                           diagonal_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)-1,
                           point(1),
                           current_val,
                           adjacent_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)+1,
                           point(1),
                           current_val,
                           adjacent_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)-1,
                           point(1)+1,
                           current_val,
                           diagonal_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0),
                           point(1)+1,
                           current_val,
                           adjacent_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);

      touchObstacleSearchCell(grid_data,
                           expl_layer,
                           point,
                           point(0)+1,
                           point(1)+1,
                           current_val,
                           diagonal_dist,
                           obstacle_cells,
                           frontier_cells,
                           point_queue);
    }

    return true;
  }

  
} /* namespace */
