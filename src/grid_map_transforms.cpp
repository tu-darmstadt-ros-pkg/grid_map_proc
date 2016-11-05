#include <grid_map_proc/grid_map_transforms.h>


#include <opencv2/highgui/highgui.hpp>

namespace grid_map_transforms{
  
  
  bool addDistanceTransform(grid_map::GridMap& grid_map,
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
  
} /* namespace */
