#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// CORRECT: ROS2 generates lowercase with underscores
#include "robot_description/srv/plan_path.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <memory>
#include <limits>

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner() : rclcpp::Node("astar_planner"), 
                     tf_buffer_(this->get_clock()), 
                     tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "=== A* Path Planner Constructor Started ===");
        
        RCLCPP_INFO(this->get_logger(), "Creating map subscription...");
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Creating AMCL pose subscription...");
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&AStarPlanner::poseCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Creating path publisher...");
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        
        RCLCPP_INFO(this->get_logger(), "Creating path planning service...");
        
        // CORRECT: Use lowercase with underscores
        plan_service_ = this->create_service<robot_description::srv::PlanPath>(
            "/plan_path", 
            std::bind(&AStarPlanner::planPathCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "✓ Service /plan_path created successfully!");
        
        map_received_ = false;
        current_pose_received_ = false;
        
        RCLCPP_INFO(this->get_logger(), "=== A* Path Planner node started successfully ===");
    }

private:
    struct Node
    {
        int x, y;
        double g_cost;
        double f_cost;
        Node* parent;

        Node(int x, int y) : x(x), y(y), 
                            g_cost(std::numeric_limits<double>::max()),
                            f_cost(std::numeric_limits<double>::max()), 
                            parent(nullptr) {}
    };

    struct NodeHash
    {
        std::size_t operator()(const std::pair<int, int>& p) const
        {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!msg) return;
        
        map_ = msg;
        map_received_ = true;
        resolution_ = msg->info.resolution;
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        width_ = msg->info.width;
        height_ = msg->info.height;
        
        RCLCPP_INFO(this->get_logger(), "✓ Map received: %dx%d, resolution: %f", 
                    width_, height_, resolution_);
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!msg) return;
        
        current_pose_ = msg->pose.pose;
        if (!current_pose_received_) {
            RCLCPP_INFO(this->get_logger(), "✓ First AMCL pose received: (%.2f, %.2f)", 
                        current_pose_.position.x, current_pose_.position.y);
            current_pose_received_ = true;
        }
    }

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const
    {
        if (!map_received_) return false;
        
        gx = static_cast<int>((wx - origin_x_) / resolution_);
        gy = static_cast<int>((wy - origin_y_) / resolution_);
        
        return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
    }

    void gridToWorld(int gx, int gy, double& wx, double& wy) const
    {
        wx = gx * resolution_ + origin_x_ + resolution_ / 2.0;
        wy = gy * resolution_ + origin_y_ + resolution_ / 2.0;
    }

    bool isCellFree(int x, int y) const
    {
        if (!map_received_ || x < 0 || x >= width_ || y < 0 || y >= height_) {
            return false;
        }
        
        int index = y * width_ + x;
        if (index >= static_cast<int>(map_->data.size())) return false;
        
        int cost = map_->data[index];
        return cost == 0 || cost == -1;
    }

    double heuristic(int x1, int y1, int x2, int y2) const
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const
    {
        std::vector<std::pair<int, int>> neighbors;
        std::vector<std::pair<int, int>> directions = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},           {0, 1},
            {1, -1},  {1, 0},  {1, 1}
        };
        
        for (const auto& dir : directions) {
            int nx = x + dir.first;
            int ny = y + dir.second;
            if (isCellFree(nx, ny)) {
                neighbors.push_back({nx, ny});
            }
        }
        return neighbors;
    }

    nav_msgs::msg::Path planPath(const geometry_msgs::msg::Pose& start, 
                                 const geometry_msgs::msg::Pose& goal)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "Map not received yet");
            return path;
        }

        // Convert to grid coordinates
        int start_x, start_y, goal_x, goal_y;
        if (!worldToGrid(start.position.x, start.position.y, start_x, start_y)) {
            RCLCPP_ERROR(this->get_logger(), "Start pose out of map bounds");
            return path;
        }
        if (!worldToGrid(goal.position.x, goal.position.y, goal_x, goal_y)) {
            RCLCPP_ERROR(this->get_logger(), "Goal pose out of map bounds");
            return path;
        }

        // A* algorithm
        auto cmp = [](Node* a, Node* b) { return a->f_cost > b->f_cost; };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);
        
        std::unordered_map<std::pair<int, int>, Node*, NodeHash> all_nodes;
        std::unordered_map<std::pair<int, int>, double, NodeHash> g_costs;

        // Start node
        Node* start_node = new Node(start_x, start_y);
        start_node->g_cost = 0.0;
        start_node->f_cost = heuristic(start_x, start_y, goal_x, goal_y);
        open_set.push(start_node);
        all_nodes[{start_x, start_y}] = start_node;
        g_costs[{start_x, start_y}] = 0.0;

        Node* goal_node = nullptr;
        int iterations = 0;
        const int MAX_ITERATIONS = 100000;

        while (!open_set.empty() && iterations++ < MAX_ITERATIONS) {
            Node* current = open_set.top();
            open_set.pop();

            if (current->x == goal_x && current->y == goal_y) {
                goal_node = current;
                break;
            }

            for (const auto& neighbor : getNeighbors(current->x, current->y)) {
                int nx = neighbor.first;
                int ny = neighbor.second;
                
                // Calculate movement cost (sqrt(2) for diagonal, 1 for straight)
                double dx = nx - current->x;
                double dy = ny - current->y;
                double move_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
                double tentative_g = current->g_cost + move_cost;

                auto it = g_costs.find({nx, ny});
                if (it == g_costs.end() || tentative_g < it->second) {
                    g_costs[{nx, ny}] = tentative_g;

                    Node* neighbor_node = all_nodes[{nx, ny}];
                    if (!neighbor_node) {
                        neighbor_node = new Node(nx, ny);
                        all_nodes[{nx, ny}] = neighbor_node;
                    }

                    neighbor_node->g_cost = tentative_g;
                    neighbor_node->f_cost = tentative_g + heuristic(nx, ny, goal_x, goal_y);
                    neighbor_node->parent = current;
                    open_set.push(neighbor_node);
                }
            }
        }

        if (iterations >= MAX_ITERATIONS) {
            RCLCPP_WARN(this->get_logger(), "A* exceeded maximum iterations");
        }

        // Reconstruct path
        if (goal_node) {
            std::vector<Node*> path_nodes;
            Node* current = goal_node;
            while (current != nullptr) {
                path_nodes.push_back(current);
                current = current->parent;
            }
            std::reverse(path_nodes.begin(), path_nodes.end());

            for (Node* node : path_nodes) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = this->now();
                
                double wx, wy;
                gridToWorld(node->x, node->y, wx, wy);
                
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                
                path.poses.push_back(pose);
            }
            
            RCLCPP_INFO(this->get_logger(), "✓ Path found with %zu waypoints", path.poses.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ No path found");
        }

        // Cleanup
        for (auto& pair : all_nodes) {
            delete pair.second;
        }

        return path;
    }

    // CORRECT: Use PlanPath (not PathPlan)
    void planPathCallback(
    const std::shared_ptr<robot_description::srv::PlanPath::Request> request,
    std::shared_ptr<robot_description::srv::PlanPath::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "=== Path planning service called ===");
    RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", 
                request->goal.pose.position.x, request->goal.pose.position.y);
    
    // Check if map is available
    if (!map_received_) {
        RCLCPP_ERROR(this->get_logger(), "❌ Cannot plan path: No map received yet!");
        response->path.header.frame_id = "map";
        response->path.header.stamp = this->now();
        return;
    }
    
    // Get start pose
    geometry_msgs::msg::Pose start_pose;
    
    if (current_pose_received_) {
        start_pose = current_pose_;
        RCLCPP_INFO(this->get_logger(), "✓ Using AMCL pose as start: (%.2f, %.2f)", 
                    start_pose.position.x, start_pose.position.y);
    } else {
        RCLCPP_WARN(this->get_logger(), "AMCL pose not received, trying TF lookup...");
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            
            start_pose.position.x = transform.transform.translation.x;
            start_pose.position.y = transform.transform.translation.y;
            start_pose.position.z = transform.transform.translation.z;
            start_pose.orientation = transform.transform.rotation;
            
            RCLCPP_INFO(this->get_logger(), "✓ Using TF pose as start: (%.2f, %.2f)", 
                        start_pose.position.x, start_pose.position.y);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "❌ Could not get robot pose: %s", ex.what());
            response->path.header.frame_id = "map";
            response->path.header.stamp = this->now();
            return;
        }
    }
    
    // Compute path
    RCLCPP_INFO(this->get_logger(), "🚀 Computing path from (%.2f, %.2f) to (%.2f, %.2f)...",
                start_pose.position.x, start_pose.position.y,
                request->goal.pose.position.x, request->goal.pose.position.y);
    
    response->path = planPath(start_pose, request->goal.pose);
    
    // Publish for visualization
    if (!response->path.poses.empty()) {
        path_pub_->publish(response->path);
        RCLCPP_INFO(this->get_logger(), "✅ Path published to /global_path with %zu waypoints", 
                   response->path.poses.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️  Empty path returned - no valid path found");
    }
}

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // CORRECT: Use PlanPath
    rclcpp::Service<robot_description::srv::PlanPath>::SharedPtr plan_service_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::Pose current_pose_;
    bool map_received_;
    bool current_pose_received_;

    double resolution_;
    double origin_x_, origin_y_;
    int width_, height_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}