// SPDX-License-Identifier: Apache-2.0
// Host-only protocol checks: no rclcpp::init(), mapper, CUDA calls or robot I/O.
#include "mesh_visualization.h"
#include <iostream>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
template<class F> void rejected(F operation) {
  bool caught = false;
  try { operation(); } catch (const std::runtime_error &) { caught = true; }
  require(caught, "Malformed Mesh was accepted");
}
int main() {
  try {
    nvblox_msgs::msg::Mesh mesh;
    mesh.header.frame_id = "base_link";
    mesh.block_size_m = .24f;
    nvblox_msgs::msg::Index3D index; index.x = 1;
    mesh.block_indices.push_back(index); mesh.blocks.resize(1);
    auto &block = mesh.blocks[0];
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::Point32 point;
      point.x = .25f; point.y = .01f * i; point.z = .1f + .01f * (i == 2);
      block.vertices.push_back(point);
      std_msgs::msg::ColorRGBA color;
      color.r = i == 0; color.g = i == 1; color.b = i == 2; color.a = .2f;
      block.colors.push_back(color);
    }
    block.triangles = {2, 0, 1};
    const nvblox::Vector3f center(0, 0, .5f), size(2, 2, 1.5f);
    auto convert = [&](const auto &msg) { return meshMarkersFromMessage(msg, center, size); };
    auto markers = convert(mesh);
    require(markers.markers.size() == 1, "Triangle disappeared");
    const auto &marker = markers.markers[0];
    require(marker.ns == "1_0_0" && marker.id == 0, "Unstable block identity");
    require(marker.points.size() == 3 && marker.colors.size() == 3, "Triangle/color expansion broken");
    require(std::abs(marker.points[0].x - .25) < 1e-6, "Block origin incorrectly added to GLOBAL vertex");
    require(std::abs(marker.points[0].y - .02) < 1e-6 && marker.colors[0].b == 1,
            "Triangle indices not applied to vertices/colors consistently");
    require(marker.colors[0].a == 1 && marker.scale.x == 1, "Opacity/scale changed geometry view");
    auto deletion = mesh; deletion.clear = true; deletion.blocks[0].triangles.clear();
    markers = convert(deletion);
    require(markers.markers.size() == 2 &&
            markers.markers[0].action == visualization_msgs::msg::Marker::DELETEALL &&
            markers.markers[1].action == visualization_msgs::msg::Marker::DELETE,
            "Full clear / deleted block protocol broken");
    auto far = mesh; far.block_indices[0].x = 100;
    require(convert(far).markers.empty(), "ROI block exclusion broken");
    auto no_colors = mesh; no_colors.blocks[0].colors.clear();
    require(convert(no_colors).markers[0].colors.empty() &&
            convert(no_colors).markers[0].color.a == 1, "Fallback color broken");
    auto malformed = mesh; malformed.blocks[0].triangles[0] = -1;
    rejected([&] { convert(malformed); });
    malformed = mesh; malformed.blocks[0].colors.pop_back();
    rejected([&] { convert(malformed); });
    malformed = mesh; malformed.block_indices.clear();
    rejected([&] { convert(malformed); });
    std::cout << "PASS: native global coordinates, triangle/RGB correspondence, ROI, clear/delete and malformed Mesh rejection. No ROS/GPU/motion executed.\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
