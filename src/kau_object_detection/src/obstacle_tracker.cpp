// Copyright 2026 KAU AMET Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kau_object_detection/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace kau_object_detection
{

namespace
{

/// One admissible candidate-to-track link. Only pairs inside the association
/// gate are ever built, so the greedy pass never has to re-check the gate.
struct AssociationPair
{
  double distance_m{0.0};
  std::size_t track_index{0U};
  std::size_t measurement_index{0U};
};

double planar_distance(
  const double first_x,
  const double first_y,
  const double second_x,
  const double second_y)
{
  const double delta_x = first_x - second_x;
  const double delta_y = first_y - second_y;
  return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

bool measurement_is_finite(const ObstacleMeasurement & measurement)
{
  return std::isfinite(measurement.x_m) && std::isfinite(measurement.y_m) &&
         std::isfinite(measurement.radius_m);
}

/// Keeps the tracker predictable when a caller hands over an unusable value.
/// Clamping rather than throwing matters here because the node builds these
/// options from YAML, where a wrong sign must not take the perception node
/// down mid-run.
ObstacleTrackerOptions clamp_options(const ObstacleTrackerOptions & options)
{
  ObstacleTrackerOptions clamped = options;

  if (!std::isfinite(clamped.association_distance_m) || clamped.association_distance_m < 0.0) {
    clamped.association_distance_m = 0.0;
  }
  if (!std::isfinite(clamped.outlier_distance_m) || clamped.outlier_distance_m < 0.0) {
    clamped.outlier_distance_m = 0.0;
  }
  if (!std::isfinite(clamped.smoothing_alpha)) {
    clamped.smoothing_alpha = 1.0;
  }
  clamped.smoothing_alpha = std::min(std::max(clamped.smoothing_alpha, 0.0), 1.0);
  if (clamped.minimum_confirmation_frames < 1U) {
    clamped.minimum_confirmation_frames = 1U;
  }
  if (!std::isfinite(clamped.track_timeout_s) || clamped.track_timeout_s < 0.0) {
    clamped.track_timeout_s = 0.0;
  }

  return clamped;
}

}  // namespace

ObstacleTracker::ObstacleTracker()
: ObstacleTracker(ObstacleTrackerOptions{})
{
}

ObstacleTracker::ObstacleTracker(const ObstacleTrackerOptions & options)
: options_(clamp_options(options))
{
}

void ObstacleTracker::reset()
{
  tracks_.clear();
  has_timestamp_ = false;
  last_timestamp_s_ = 0.0;
}

std::size_t ObstacleTracker::active_track_count() const
{
  return tracks_.size();
}

const ObstacleTrackerOptions & ObstacleTracker::options() const
{
  return options_;
}

ObstacleTrackerUpdate ObstacleTracker::update(
  const std::vector<ObstacleMeasurement> & measurements,
  const double timestamp_s)
{
  ObstacleTrackerUpdate result;

  // A frame that cannot be placed on the timeline invalidates every age the
  // timeout rule depends on, so the safe answer is to start over rather than to
  // keep tracks that may be arbitrarily old.
  if (!std::isfinite(timestamp_s) || (has_timestamp_ && timestamp_s < last_timestamp_s_)) {
    reset();
    result.reset_performed = true;
  }

  if (!std::isfinite(timestamp_s)) {
    // Nothing in this frame can be timestamped, so no track may be created from
    // it either. The tracker is already empty at this point.
    return result;
  }

  // Timeout pruning runs before association so a measurement can never be
  // linked to a track that has already expired.
  const std::size_t tracks_before_timeout = tracks_.size();
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this, timestamp_s](const Track & track) {
        return (timestamp_s - track.last_observed_s) > options_.track_timeout_s;
      }),
    tracks_.end());
  result.timed_out_tracks = tracks_before_timeout - tracks_.size();

  for (auto & track : tracks_) {
    track.observed_this_frame = false;
    track.measurement_index = 0U;
  }

  std::vector<bool> measurement_used(measurements.size(), false);
  for (std::size_t index = 0U; index < measurements.size(); ++index) {
    // A non-finite candidate is dropped outright: it can neither update a track
    // nor found one.
    if (!measurement_is_finite(measurements[index])) {
      measurement_used[index] = true;
    }
  }

  // Deterministic greedy 1:1 association. Every admissible pair is enumerated,
  // ordered by distance, and consumed closest first; ties break on the track id
  // and then on the measurement index so the result never depends on iteration
  // order or on floating-point luck.
  std::vector<AssociationPair> pairs;
  pairs.reserve(tracks_.size() * measurements.size());
  for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
    for (std::size_t index = 0U; index < measurements.size(); ++index) {
      if (measurement_used[index]) {
        continue;
      }
      const double distance = planar_distance(
        tracks_[track_index].x_m, tracks_[track_index].y_m,
        measurements[index].x_m, measurements[index].y_m);
      if (distance > options_.association_distance_m) {
        continue;
      }
      pairs.push_back(AssociationPair{distance, track_index, index});
    }
  }

  std::sort(
    pairs.begin(), pairs.end(),
    [this](const AssociationPair & first, const AssociationPair & second) {
      if (first.distance_m != second.distance_m) {
        return first.distance_m < second.distance_m;
      }
      if (tracks_[first.track_index].id != tracks_[second.track_index].id) {
        return tracks_[first.track_index].id < tracks_[second.track_index].id;
      }
      return first.measurement_index < second.measurement_index;
    });

  std::vector<bool> track_linked(tracks_.size(), false);
  for (const auto & pair : pairs) {
    if (track_linked[pair.track_index] || measurement_used[pair.measurement_index]) {
      continue;
    }
    track_linked[pair.track_index] = true;
    measurement_used[pair.measurement_index] = true;

    auto & track = tracks_[pair.track_index];
    const auto & measurement = measurements[pair.measurement_index];

    // Outlier gate. Association only decides which track a measurement belongs
    // to; whether that measurement is credible enough to move the estimate is a
    // separate decision, taken here against the previous stable estimate.
    if (pair.distance_m > options_.outlier_distance_m) {
      ++result.rejected_outliers;
      continue;
    }

    const double alpha = options_.smoothing_alpha;
    track.x_m = (alpha * measurement.x_m) + ((1.0 - alpha) * track.x_m);
    track.y_m = (alpha * measurement.y_m) + ((1.0 - alpha) * track.y_m);
    // The radius is carried, never smoothed: it is a configured cone dimension
    // in this stage, not an estimated quantity.
    track.radius_m = measurement.radius_m;
    track.last_observed_s = timestamp_s;
    track.observed_this_frame = true;
    track.measurement_index = pair.measurement_index;
    ++track.observation_count;
    ++result.confirmed_observations;
  }

  // Every measurement that found no track starts one, initialised with the
  // measurement itself. Unconfirmed tracks stay internal until they collect
  // `minimum_confirmation_frames` observations.
  for (std::size_t index = 0U; index < measurements.size(); ++index) {
    if (measurement_used[index]) {
      continue;
    }
    Track track;
    track.id = next_track_id_++;
    track.x_m = measurements[index].x_m;
    track.y_m = measurements[index].y_m;
    track.radius_m = measurements[index].radius_m;
    track.observation_count = 1U;
    track.last_observed_s = timestamp_s;
    track.observed_this_frame = true;
    track.measurement_index = index;
    tracks_.push_back(track);
    ++result.new_tracks;
    ++result.confirmed_observations;
  }

  std::sort(
    tracks_.begin(), tracks_.end(),
    [](const Track & first, const Track & second) {
      return first.id < second.id;
    });

  result.active_tracks = tracks_.size();
  result.obstacles.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    // Both conditions are required: a track that survived a dropout but was not
    // measured in this frame must never be published as an observation, and a
    // fresh track must not be published before it is confirmed.
    if (!track.observed_this_frame) {
      continue;
    }
    if (track.observation_count < options_.minimum_confirmation_frames) {
      continue;
    }

    SmoothedObstacle obstacle;
    obstacle.track_id = track.id;
    obstacle.x_m = track.x_m;
    obstacle.y_m = track.y_m;
    obstacle.radius_m = track.radius_m;
    obstacle.measurement_index = track.measurement_index;
    obstacle.observation_count = track.observation_count;
    result.obstacles.push_back(obstacle);
  }

  last_timestamp_s_ = timestamp_s;
  has_timestamp_ = true;

  return result;
}

}  // namespace kau_object_detection
