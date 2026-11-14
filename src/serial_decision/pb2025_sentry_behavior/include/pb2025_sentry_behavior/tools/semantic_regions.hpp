#ifndef PB2025_SENTRY_BEHAVIOR__SEMANTIC_REGIONS_HPP_
#define PB2025_SENTRY_BEHAVIOR__SEMANTIC_REGIONS_HPP_

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pb2025_sentry_behavior
{
class SemanticRegions
{
public:
  bool load(const std::string & path, const std::vector<std::string> & selected, std::string & error);
  bool contains(double x, double y, double minimum_boundary_distance = 0.0) const;

private:
  std::vector<std::vector<std::pair<double, double>>> polygons_;
};

inline bool SemanticRegions::load(
  const std::string & path, const std::vector<std::string> & selected, std::string & error)
{
  polygons_.clear();
  try {
    const std::set<std::string> wanted(selected.begin(), selected.end());

    if (path.empty() || wanted.empty() || wanted.size() != selected.size()) {
      throw std::runtime_error("semantic map and distinct region IDs are required");
    }

    std::set<std::string> found;

    std::vector<std::vector<std::pair<double, double>>> polygons;

    auto add = [&](const std::string & id, std::vector<std::pair<double, double>> points) {
      if (!wanted.count(id)) return;

      if (!found.insert(id).second || points.size() < 3) {
        throw std::runtime_error("duplicate ID or invalid polygon: " + id);
      }
      for (const auto & p : points) {
        if (!std::isfinite(p.first) || !std::isfinite(p.second)) {
          throw std::runtime_error("non-finite point in region: " + id);
        }
      }

      double twice_area = 0;

      for (size_t i = 0; i < points.size(); ++i) {
        const auto & a = points[i];

        const auto & b = points[(i + 1) % points.size()];
        twice_area += a.first * b.second - b.first * a.second;
      }
      if (std::abs(twice_area) < 1e-9) 
      throw std::runtime_error("zero-area polygon: " + id);

      polygons.push_back(std::move(points));
    };

    const auto extension = std::filesystem::path(path).extension().string();

    if (extension != ".yaml" && extension != ".yml") {
      throw std::runtime_error("semantic map must be a YAML file");
    }

    const auto regions = YAML::LoadFile(path)["regions"];

    if (!regions || !regions.IsSequence()) {
      throw std::runtime_error("missing regions list");
    }

    for (const auto & region : regions) {

      const auto id = region["id"].as<std::string>();

      if (!wanted.count(id)) {
        continue;
      }
      if (region["frame_id"].as<std::string>() != "map" || !region["points"].IsSequence()) {
        throw std::runtime_error("region must have map frame and points: " + id);
      }

      std::vector<std::pair<double, double>> points;

      for (const auto & point : region["points"]) {
        points.emplace_back(point["x"].as<double>(), point["y"].as<double>());
      }

      add(id, std::move(points));
    }
    if (found != wanted) throw std::runtime_error("selected region ID not found");

    polygons_ = std::move(polygons);

    error.clear();

    return true;
  } catch (const std::exception & ex) {
    error = ex.what();

    return false;
  }
}

inline bool SemanticRegions::contains(
  double x, double y, double minimum_boundary_distance) const
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(minimum_boundary_distance) || minimum_boundary_distance < 0.0)
  {
    return false;
  }

  const double minimum_distance_squared = minimum_boundary_distance * minimum_boundary_distance;

  for (const auto & polygon : polygons_) {
    bool inside = false;
    bool on_boundary = false;
    double nearest_boundary_squared = std::numeric_limits<double>::infinity();

    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const auto & a = polygon[j];
      const auto & b = polygon[i];

      const double dx = b.first - a.first, dy = b.second - a.second;
      const double length_squared = dx * dx + dy * dy;
      const double projection = length_squared > 0.0 ?
        std::clamp(((x - a.first) * dx + (y - a.second) * dy) / length_squared, 0.0, 1.0) : 0.0;
      const double nearest_x = a.first + projection * dx;
      const double nearest_y = a.second + projection * dy;
      const double offset_x = x - nearest_x;
      const double offset_y = y - nearest_y;
      nearest_boundary_squared = std::min(
        nearest_boundary_squared, offset_x * offset_x + offset_y * offset_y);

      if (offset_x * offset_x + offset_y * offset_y <= 1e-18) on_boundary = true;
      if ((a.second > y) != (b.second > y) &&
        x < dx * (y - a.second) / dy + a.first)
      {
        inside = !inside;
      }
    }

    inside = inside || on_boundary;
    if (inside && nearest_boundary_squared >= minimum_distance_squared) return true;
  }
  return false;
}
}

#endif  // PB2025_SENTRY_BEHAVIOR__SEMANTIC_REGIONS_HPP_
