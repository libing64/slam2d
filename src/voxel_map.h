#ifndef VOXEL_MAP_H
#define VOXEL_MAP_H

#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <tuple>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace Eigen;

// Utility functions for log-odds conversion
inline float probToLogOdds(float prob) {
    return std::log(prob / (1.0f - prob));
}

inline float logOddsToProb(float odds) {
    return 1.0f - (1.0f / (1.0f + std::exp(odds)));
}

class VoxelMap2D {
public:
    int width, height;
    double resolution;
    Vector2d origin_offset; // Physical position of the (0,0) pixel

    std::vector<float> data; // Log-odds representation

    const float p_hit = 0.60f;
    const float p_miss = 0.45f;
    const float l_hit = probToLogOdds(p_hit);
    const float l_miss = probToLogOdds(p_miss);
    const float l_min = probToLogOdds(0.12f);
    const float l_max = probToLogOdds(0.97f);

    VoxelMap2D(int w, int h, double res) : width(w), height(h), resolution(res) {
        data.resize(w * h, 0.0f); // 0 log-odds means 0.5 probability (unknown)
        origin_offset = Vector2d(-w * res * 0.5, -h * res * 0.5);
    }

    bool isInside(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    void setCell(int x, int y, float val) {
        if (isInside(x, y)) {
            data[y * width + x] = val;
        }
    }

    float getCell(int x, int y) const {
        if (isInside(x, y)) {
            return data[y * width + x];
        }
        return 0.0f; // unknown
    }

    float getProb(int x, int y) const {
        return logOddsToProb(getCell(x, y));
    }

    // Convert world physical coordinates to map pixel indices
    Vector2i worldToMap(const Vector2d& p) const {
        int x = std::round((p.x() - origin_offset.x()) / resolution);
        int y = std::round((p.y() - origin_offset.y()) / resolution);
        return Vector2i(x, y);
    }

    // Convert map pixel indices to world physical coordinates
    Vector2d mapToWorld(int x, int y) const {
        return Vector2d(x * resolution + origin_offset.x(), y * resolution + origin_offset.y());
    }

    // Bilinear interpolation for sub-pixel precision
    float getProbBilinear(const Vector2d& p) const {
        double px = (p.x() - origin_offset.x()) / resolution;
        double py = (p.y() - origin_offset.y()) / resolution;

        int x0 = std::floor(px);
        int y0 = std::floor(py);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        double dx = px - x0;
        double dy = py - y0;

        float p00 = getProb(x0, y0);
        float p10 = getProb(x1, y0);
        float p01 = getProb(x0, y1);
        float p11 = getProb(x1, y1);

        float p0 = p00 * (1.0 - dx) + p10 * dx;
        float p1 = p01 * (1.0 - dx) + p11 * dx;

        return p0 * (1.0 - dy) + p1 * dy;
    }

    void updateCell(int x, int y, bool hit) {
        if (isInside(x, y)) {
            int idx = y * width + x;
            data[idx] += hit ? l_hit : l_miss;
            data[idx] = std::max(l_min, std::min(l_max, data[idx]));
        }
    }

    void bresenham(const Vector2i& p1, const Vector2i& p2) {
        int x1 = p1.x(), y1 = p1.y();
        int x2 = p2.x(), y2 = p2.y();
        int dx = std::abs(x2 - x1);
        int sx = x1 < x2 ? 1 : -1;
        int dy = std::abs(y2 - y1);
        int sy = y1 < y2 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2;
        int e2;

        while (true) {
            // Treat the end point as hit, other points as miss
            if (x1 == x2 && y1 == y2) {
                updateCell(x1, y1, true); // Hit
                break;
            } else {
                updateCell(x1, y1, false); // Miss
            }

            e2 = err;
            if (e2 > -dx) { err -= dy; x1 += sx; }
            if (e2 < dy) { err += dx; y1 += sy; }
        }
    }
};

// Precomputed Multi-resolution Map (Grid Pyramid) for B2B
class GridPyramid {
public:
    std::vector<std::vector<float>> grids; // levels: 0 is highest resolution
    int base_width, base_height;
    int num_levels;

    GridPyramid(const VoxelMap2D& base_map, int levels) {
        num_levels = levels;
        base_width = base_map.width;
        base_height = base_map.height;
        grids.resize(num_levels);

        // Level 0 is the original probability map
        grids[0].resize(base_width * base_height);
        for (int y = 0; y < base_height; y++) {
            for (int x = 0; x < base_width; x++) {
                grids[0][y * base_width + x] = base_map.getProb(x, y);
            }
        }

        // Build lower resolution maps using max-pooling
        for (int l = 1; l < num_levels; ++l) {
            int prev_w = base_width >> (l - 1);
            int prev_h = base_height >> (l - 1);
            int curr_w = base_width >> l;
            int curr_h = base_height >> l;
            grids[l].resize(curr_w * curr_h);

            for (int y = 0; y < curr_h; ++y) {
                for (int x = 0; x < curr_w; ++x) {
                    float max_val = 0.0f;
                    // Check 2x2 block in the previous level
                    for (int dy = 0; dy < 2; ++dy) {
                        for (int dx = 0; dx < 2; ++dx) {
                            int px = x * 2 + dx;
                            int py = y * 2 + dy;
                            if (px < prev_w && py < prev_h) {
                                float val = grids[l - 1][py * prev_w + px];
                                if (val > max_val) max_val = val;
                            }
                        }
                    }
                    grids[l][y * curr_w + x] = max_val;
                }
            }
        }
    }

    float getProbMax(int level, int x, int y) const {
        int w = base_width >> level;
        int h = base_height >> level;
        if (x >= 0 && x < w && y >= 0 && y < h) {
            return grids[level][y * w + x];
        }
        return 0.0f;
    }
};

// Search Node for B2B
struct SearchNode {
    int x_idx, y_idx, theta_idx;
    int level;
    float score;

    SearchNode(int x, int y, int t, int l, float s)
        : x_idx(x), y_idx(y), theta_idx(t), level(l), score(s) {}

    bool operator<(const SearchNode& other) const {
        return score < other.score; // Max-heap
    }
};

class BranchAndBoundMatcher {
public:
    double search_window_x = 0.2; // meters
    double search_window_y = 0.2;
    double search_window_theta = 0.1; // radians

    double resolution;
    double angular_resolution;

    BranchAndBoundMatcher(double res, double a_res) : resolution(res), angular_resolution(a_res) {}

    // Score a single pose at a specific pyramid level
    float scorePose(const GridPyramid& pyramid, const pcl::PointCloud<pcl::PointXY>& scan,
                    const Vector2d& origin_offset, int level, double x, double y, double theta) {
        float score = 0.0f;
        int step = 1 << level;
        double level_res = resolution * step;

        double cos_th = std::cos(theta);
        double sin_th = std::sin(theta);

        for (const auto& pt : scan.points) {
            // Transform point
            double p_world_x = cos_th * pt.x - sin_th * pt.y + x;
            double p_world_y = sin_th * pt.x + cos_th * pt.y + y;

            // Map to grid indices
            int idx_x = std::round((p_world_x - origin_offset.x()) / level_res);
            int idx_y = std::round((p_world_y - origin_offset.y()) / level_res);

            score += pyramid.getProbMax(level, idx_x, idx_y);
        }
        return score;
    }

    Vector3d match(const VoxelMap2D& map, const pcl::PointCloud<pcl::PointXY>& scan, const Vector3d& initial_pose) {
        int num_levels = 4;
        GridPyramid pyramid(map, num_levels);

        int max_x_idx = std::ceil(search_window_x / resolution);
        int max_y_idx = std::ceil(search_window_y / resolution);
        int max_th_idx = std::ceil(search_window_theta / angular_resolution);

        std::priority_queue<SearchNode> pq;

        // Populate initial nodes at the highest level
        int top_level = num_levels - 1;
        int step = 1 << top_level;

        for (int th = -max_th_idx; th <= max_th_idx; ++th) {
            double theta = initial_pose(0) + th * angular_resolution;
            for (int y = -max_y_idx; y <= max_y_idx; y += step) {
                for (int x = -max_x_idx; x <= max_x_idx; x += step) {
                    double px = initial_pose(1) + x * resolution;
                    double py = initial_pose(2) + y * resolution;

                    float score = scorePose(pyramid, scan, map.origin_offset, top_level, px, py, theta);
                    pq.emplace(x, y, th, top_level, score);
                }
            }
        }

        Vector3d best_pose = initial_pose;
        float best_score = -1.0f;

        while (!pq.empty()) {
            SearchNode curr = pq.top();
            pq.pop();

            // If the upper bound score of this node is worse than our current best, prune
            if (curr.score < best_score) {
                break;
            }

            // If we are at the bottom level, this is an exact score
            if (curr.level == 0) {
                best_score = curr.score;
                best_pose(0) = initial_pose(0) + curr.theta_idx * angular_resolution;
                best_pose(1) = initial_pose(1) + curr.x_idx * resolution;
                best_pose(2) = initial_pose(2) + curr.y_idx * resolution;
                // Since this is the highest score popped and level == 0, it is the optimal
                // because all other nodes in the queue have an upper bound <= curr.score
            } else {
                // Branch into 4 children (split x and y)
                int next_level = curr.level - 1;
                int step = 1 << next_level;

                for (int dy = 0; dy < 2; ++dy) {
                    for (int dx = 0; dx < 2; ++dx) {
                        int nx = curr.x_idx + dx * step;
                        int ny = curr.y_idx + dy * step;

                        // Ensure we don't go out of original search bounds
                        if (std::abs(nx) > max_x_idx || std::abs(ny) > max_y_idx) continue;

                        double px = initial_pose(1) + nx * resolution;
                        double py = initial_pose(2) + ny * resolution;
                        double theta = initial_pose(0) + curr.theta_idx * angular_resolution;

                        float score = scorePose(pyramid, scan, map.origin_offset, next_level, px, py, theta);
                        
                        // Pruning before pushing
                        if (score > best_score) {
                            pq.emplace(nx, ny, curr.theta_idx, next_level, score);
                        }
                    }
                }
            }
        }

        return best_pose;
    }
};

#endif // VOXEL_MAP_H