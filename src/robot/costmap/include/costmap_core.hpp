#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    double resolution_; // meters per cell
    int width, height; // number of cells in x and y directions (40m x 40m area)
    double origin_x_, origin_y_; // origin of the costmap in world coordinates
    double inflation_radius; // radius for obstacle inflation in meters
    int8_t max_inflation_cost = 100; // maximum cost for inflated obstacles

    void reset();

    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void setCell(int x_grid, int y_grid, int8_t value);
    void inflateObstacles();

    std::vector<int8_t> getCostmap() const { return costmap_; }

  private:
    rclcpp::Logger logger_;

    std::vector<int8_t> costmap_; // 1D vector to represent the 2D costmap
};

}  

#endif  