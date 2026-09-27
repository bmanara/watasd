#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
    resolution_ = 0.1; // meters per cell
    width = 400; // number of cells in x direction (40m area)
    height = 400; // number of cells in y direction (40m area)

    // So that robot is at (0, 0) in costmap coordinates
    origin_x_ = -20.0; // origin of the costmap in world coordinates
    origin_y_ = -20.0; // origin of the costmap in world coordinates

    inflation_radius = 2.0; // radius for obstacle inflation in meters
    max_inflation_cost = 100; // maximum cost for inflated obstacles

    this->reset();
}

void CostmapCore::reset() {
    costmap_.assign(width * height, 0); // Set all cells to free space (0)
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * cos(angle);
  double y = range * sin(angle);

  x_grid = static_cast<int>((x - this->origin_x_) / this->resolution_);
  y_grid = static_cast<int>((y - this->origin_y_) / this->resolution_);
}

void CostmapCore::setCell(int x_grid, int y_grid, int8_t value) {
    if (x_grid >= 0 && x_grid < this->width && y_grid >= 0 && y_grid < this->height) {
        this->costmap_[y_grid * this->width + x_grid] = value;
    } else {
        RCLCPP_WARN(logger_, "Grid coordinates out of bounds: (%d, %d)", x_grid, y_grid);
    }
}

void CostmapCore::inflateObstacles() {
    for (int y = 0; y < this->height; ++y) {
        for (int x = 0; x < this->width; ++x) {
            if (this->costmap_[y * this->width + x] == 100) {
                // If it is an obstacle, inflate around it
                int inflation_cells = static_cast<int>(this->inflation_radius / this->resolution_);
                // Inflate in a circle around the obstacle
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        // A lot of redundant checks here, but it's clear for now...
                        // There's probably a much better way to do this, but I want to be able to read my code for now
                        if (nx >= 0 && nx < this->width && ny >= 0 && ny < this->height) {
                            double distance = sqrt(dx * dx + dy * dy) * this->resolution_;
                            if (distance <= this->inflation_radius) {
                                int8_t cost = static_cast<int8_t>(
                                    std::max(0.0, this->max_inflation_cost - (distance / this->inflation_radius) * this->max_inflation_cost)
                                );

                                this->costmap_[ny * this->width + nx] = std::max(this->costmap_[ny * this->width + nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

}
