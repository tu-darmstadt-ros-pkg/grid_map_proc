#include <catch_ros/catch.hpp>
#include <grid_map_proc/grid_map_transforms.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace grid_map;
using namespace grid_map_transforms;

grid_map::GridMap createTestMap()
{
  using namespace grid_map;
  GridMap map({ "occupancy" });
  map.setGeometry(Length(1.5, 1.0), 0.01);  // 150 x 100 cells
  map["occupancy"].setZero();

  // Add rectangle obstacle
  for (int x = 30; x < 40; ++x)
    for (int y = 30; y < 70; ++y)
      map["occupancy"](x, y) = InflatedLayerProvider::OBSTACLE_VALUE;

  // Add circular obstacle
  for (int x = 60; x < 80; ++x)
    for (int y = 40; y < 60; ++y)
      if (std::hypot(x - 70, y - 50) <= 10)
        map["occupancy"](x, y) = InflatedLayerProvider::OBSTACLE_VALUE;

  // Medium-cost zone (20.0)
  for (int x = 80; x < 100; ++x)
    for (int y = 70; y < 90; ++y)
      map["occupancy"](x, y) = InflatedLayerProvider::OBSTACLE_VALUE * 0.2f;

  // High-cost zone (120.0)
  for (int x = 90; x < 110; ++x)
    for (int y = 40; y < 60; ++y)
      map["occupancy"](x, y) = InflatedLayerProvider::OBSTACLE_VALUE * 1.2f;

  return map;
}

template <typename SetupFn, typename BenchmarkFn>
double benchmarkFunction(const SetupFn& setup, const BenchmarkFn& run)
{
  constexpr int NUM_ITERATIONS = 1000;
  double total_us = 0.0;
  for (int i = 0; i < NUM_ITERATIONS; ++i)
  {
    auto context = setup();  // construct per-iteration input
    auto start = std::chrono::high_resolution_clock::now();
    run(context);  // run benchmark on it
    auto end = std::chrono::high_resolution_clock::now();
    total_us += std::chrono::duration<double, std::micro>(end - start).count();
  }
  return total_us / NUM_ITERATIONS;
}

constexpr float BINARY_INFLATION_RADIUS = 20.0f;
constexpr float SOFT_INFLATION_RADIUS = 20.0f;

TEST_CASE("Visual test: inflation layer provider", "[manual][visual]")
{
  InflatedLayerProvider inf_layer_provider;
  GridMap map = createTestMap();

  const int size_x = map.getSize()(0);  // cols
  const int size_y = map.getSize()(1);  // rows

  // Inflate layers
  constexpr int downsample_factor = 1;
  REQUIRE(inf_layer_provider.addSoftInflatedLayer(map, SOFT_INFLATION_RADIUS, "occupancy", "occupancy_soft_inflated",
                                                  downsample_factor));
  REQUIRE(inf_layer_provider.addInflatedLayer(map, BINARY_INFLATION_RADIUS, "occupancy", "occupancy_inflated",
                                              cv::MORPH_RECT));

  const auto& soft_data = map["occupancy_soft_inflated"];
  const auto& binary_data = map["occupancy_inflated"];

  // Analyze counts
  int obstacle_nonzero_count = 0;
  int soft_nonzero_count = 0;
  int binary_nonzero_count = 0;
  bool all_preserved = true;

  for (int x = 0; x < size_x; ++x)
  {
    for (int y = 0; y < size_y; ++y)
    {
      float occ_val = map["occupancy"](x, y);
      float soft_val = soft_data(x, y);
      float binary_val = binary_data(x, y);

      if (occ_val != 0.0f)
        obstacle_nonzero_count++;

      if (binary_val != 0.0f)
        binary_nonzero_count++;

      if (soft_val != 0.0f)
        soft_nonzero_count++;

      if (occ_val != 0.0f)
      {
        if (!(std::abs(soft_val - occ_val) < 1e-3f && std::abs(binary_val - occ_val) < 1e-3f))
          all_preserved = false;
      }
    }
  }

  // Preserve obstacle and medium-cost values
  REQUIRE(all_preserved);

  // The inflated layers must have more nonzero cells than the occupancy alone
  REQUIRE(soft_nonzero_count > obstacle_nonzero_count);
  REQUIRE(binary_nonzero_count > obstacle_nonzero_count);

  // Visualize the layers for debugging:

  cv::Mat occ_img(size_y, size_x, CV_8UC1);
  for (int x = 0; x < size_x; ++x)
    for (int y = 0; y < size_y; ++y)
      occ_img.at<uchar>(y, x) = static_cast<uchar>(map["occupancy"](x, y));

  cv::Mat soft_img(size_y, size_x, CV_8UC1);
  cv::Mat binary_img(size_y, size_x, CV_8UC1);
  cv::Mat combined_img(size_y, size_x, CV_8UC1);

  for (int x = 0; x < size_x; ++x)
  {
    for (int y = 0; y < size_y; ++y)
    {
      soft_img.at<uchar>(y, x) = static_cast<uchar>(soft_data(x, y));
      binary_img.at<uchar>(y, x) = static_cast<uchar>(binary_data(x, y));
      float sum = map["occupancy"](x, y) + soft_data(x, y) + binary_data(x, y);
      combined_img.at<uchar>(y, x) = static_cast<uchar>(std::min(sum * 255.0f / 300.0f, 255.0f));
    }
  }

  // Label each image
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const double scale = 0.4;
  const int thickness = 1;
  const cv::Scalar color = 255;

  cv::putText(occ_img, "Occupancy", { 5, 15 }, font, scale, color, thickness);
  cv::putText(binary_img, "Binary Inflated", { 5, 15 }, font, scale, color, thickness);
  cv::putText(soft_img, "Soft Inflated", { 5, 15 }, font, scale, color, thickness);
  cv::putText(combined_img, "Combined (Gray)", { 5, 15 }, font, scale, color, thickness);

  // Combine views
  cv::Mat row1, row2, all;
  cv::hconcat(occ_img, binary_img, row1);
  cv::hconcat(soft_img, combined_img, row2);
  cv::vconcat(row1, row2, all);

  // Save to file (e.g., in /tmp or package-relative path)
  std::string out_path = "/tmp/inflation_layers_output.png";
  cv::imwrite(out_path, all);
  std::cout << "Saved visualization to: " << out_path << std::endl;

  // Optional: Show images for visual inspection if enabled via ENV
  const char* show_env = std::getenv("SHOW_INFLATION_VISUALS");
  if (show_env && std::string(show_env) == "1")
  {
    cv::imshow("All Maps (Occupancy | Binary | Soft | Combined)", all);
    cv::waitKey(0);
  }

  SUCCEED("Visual inflation layers rendered.");
}

TEST_CASE("binary inflation kernel benchmark", "[benchmark]")
{
  InflatedLayerProvider inf_layer_provider;
  GridMap map = createTestMap();

  SECTION("MORPH_RECT")
  {
    double avg = benchmarkFunction(  //
        [&]() { return map; },
        [&](GridMap& temp) {
          inf_layer_provider.addInflatedLayer(temp, BINARY_INFLATION_RADIUS, "occupancy", "inflated", cv::MORPH_RECT);
        });
    std::cout << "[Benchmark] Binary Inflation RECT    Avg: " << avg << " µs\n";
  }

  SECTION("MORPH_ELLIPSE")
  {
    double avg = benchmarkFunction(  //
        [&]() { return map; },
        [&](GridMap& temp) {
          inf_layer_provider.addInflatedLayer(temp, BINARY_INFLATION_RADIUS, "occupancy", "inflated",
                                              cv::MORPH_ELLIPSE);
        });
    std::cout << "[Benchmark] Binary Inflation ELLIPSE Avg: " << avg << " µs\n";
  }

  SUCCEED("Inflation kernel benchmarking completed.");
}

TEST_CASE("Benchmark: Soft Inflation Decay Types (with Downsampling)", "[benchmark]")
{
  const GridMap map = createTestMap();
  InflatedLayerProvider inf_layer_provider;

  auto printBenchmarkResult = [](const std::string& decay, int ds, double avg) {
    std::cout << std::left << "[Benchmark] Soft Inflation " << std::setw(12) << decay << "DS=" << ds
              << " Avg: " << std::fixed << std::setprecision(3) << avg << " µs\n";
  };

  SECTION("Exponential Decay - Downsample x1")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 1,
                                                                    InflatedLayerProvider::DecayType::Exponential);
                          });
    printBenchmarkResult("Exponential", 1, avg);
  }

  SECTION("Quadratic Decay - Downsample x1")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 1,
                                                                    InflatedLayerProvider::DecayType::Quadratic);
                          });
    printBenchmarkResult("Quadratic", 1, avg);
  }

  SECTION("Linear Decay - Downsample x1")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 1,
                                                                    InflatedLayerProvider::DecayType::Linear);
                          });
    printBenchmarkResult("Linear", 1, avg);
  }

  SECTION("Binary Decay - Downsample x1")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 1,
                                                                    InflatedLayerProvider::DecayType::Binary);
                          });
    printBenchmarkResult("Binary", 1, avg);
  }

  SECTION("Exponential Decay - Downsample x2")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 2,
                                                                    InflatedLayerProvider::DecayType::Exponential);
                          });
    printBenchmarkResult("Exponential", 2, avg);
  }

  SECTION("Quadratic Decay - Downsample x2")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 2,
                                                                    InflatedLayerProvider::DecayType::Quadratic);
                          });
    printBenchmarkResult("Quadratic", 2, avg);
  }

  SECTION("Linear Decay - Downsample x2")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 2,
                                                                    InflatedLayerProvider::DecayType::Linear);
                          });
    printBenchmarkResult("Linear", 2, avg);
  }

  SECTION("Binary Decay - Downsample x2")
  {
    double avg =
        benchmarkFunction([&]() { return map; },
                          [&](GridMap& temp) {
                            inf_layer_provider.addSoftInflatedLayer(temp, SOFT_INFLATION_RADIUS, "occupancy", "soft", 2,
                                                                    InflatedLayerProvider::DecayType::Binary);
                          });
    printBenchmarkResult("Binary", 2, avg);
  }

  SUCCEED();
}
