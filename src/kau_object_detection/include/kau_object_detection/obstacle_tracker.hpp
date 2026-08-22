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

#ifndef KAU_OBJECT_DETECTION__OBSTACLE_TRACKER_HPP_
#define KAU_OBJECT_DETECTION__OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace kau_object_detection
{

/// Tunables of the temporal obstacle tracker. Every value is a property of the
/// measurement noise and of the update rate, never an absolute map position or
/// an assumed obstacle count.
struct ObstacleTrackerOptions
{
  /// Largest map-frame distance, in metres, at which a new measurement may be
  /// linked to an existing track. Must stay well below the smallest expected
  /// spacing between two distinct obstacles, otherwise two obstacles collapse
  /// into one track.
  double association_distance_m{0.30};

  /// Largest map-frame distance, in metres, between the previous smoothed
  /// estimate and a measurement that was already linked to that track for the
  /// measurement to be folded into the estimate. A linked measurement beyond
  /// this distance is a momentary outlier: it is discarded instead of being
  /// averaged in. Role is deliberately separate from the association gate.
  double outlier_distance_m{0.20};

  /// Weight of the newest measurement in the exponential moving average:
  /// estimate = alpha * measurement + (1 - alpha) * previous_estimate.
  /// Clamped into [0, 1] by the constructor.
  double smoothing_alpha{0.4};

  /// Number of accepted observations a track needs before it may appear in the
  /// output. Clamped to at least 1 by the constructor.
  std::size_t minimum_confirmation_frames{2U};

  /// Time, in seconds, a track survives without an accepted observation before
  /// it is deleted. Only governs the internal track store; a track that is not
  /// observed in the current frame never reaches the output regardless.
  double track_timeout_s{0.7};
};

/// One map-frame circular candidate handed to the tracker for a single frame.
/// The radius travels with the measurement only so the caller can be handed it
/// back unchanged; the tracker never smooths or alters it.
struct ObstacleMeasurement
{
  double x_m{0.0};
  double y_m{0.0};
  double radius_m{0.0};
};

/// One stabilised obstacle the tracker vouches for in the current frame.
///
/// Only tracks that were actually observed and matched in this frame are ever
/// reported, so a consumer can never receive a predicted or stale position.
struct SmoothedObstacle
{
  /// Stable identity of the track. Ids are assigned in increasing order and are
  /// never reused within one tracker lifetime.
  std::uint64_t track_id{0U};

  /// Smoothed map-frame centre.
  double x_m{0.0};
  double y_m{0.0};

  /// Radius of the measurement that produced this update, copied verbatim.
  double radius_m{0.0};

  /// Index of that measurement inside the vector passed to `update`, so the
  /// caller can recover the full provenance of the candidate.
  std::size_t measurement_index{0U};

  /// Accepted observations this track has collected, including the current one.
  std::size_t observation_count{0U};
};

/// Result of one tracker frame. The counters are diagnostics; `obstacles` is
/// the only part that is allowed to be published.
struct ObstacleTrackerUpdate
{
  /// Confirmed tracks observed in this frame, ordered by ascending track id.
  std::vector<SmoothedObstacle> obstacles;

  /// Tracks alive in the internal store after this frame, confirmed or not.
  std::size_t active_tracks{0U};

  /// Measurements accepted as an observation in this frame: linked to a track
  /// and inside the outlier gate, plus the first observation of every new
  /// track. Does not imply the track is confirmed yet.
  std::size_t confirmed_observations{0U};

  /// Measurements that were linked to a track but sat beyond the outlier gate.
  /// They updated nothing and started no track.
  std::size_t rejected_outliers{0U};

  /// Tracks created in this frame.
  std::size_t new_tracks{0U};

  /// Tracks deleted in this frame because they exceeded `track_timeout_s`.
  std::size_t timed_out_tracks{0U};

  /// True when the frame timestamp was non-finite or moved backwards and the
  /// tracker cleared its state before processing the measurements.
  bool reset_performed{false};
};

/// Temporal tracker for map-frame circular obstacle candidates.
///
/// Free of ROS and Gazebo types on purpose: the whole matching, gating and
/// smoothing behaviour is testable from a plain unit test. The tracker holds no
/// assumption about how many obstacles exist; tracks are created and deleted
/// purely from what the measurements support.
///
/// One frame runs as: timeout pruning, deterministic greedy 1:1 association
/// inside the association gate, outlier gating of every linked pair, EMA update
/// of the surviving pairs, creation of tracks for unlinked measurements, and
/// finally selection of the confirmed tracks that were observed in this frame.
class ObstacleTracker
{
public:
  ObstacleTracker();
  explicit ObstacleTracker(const ObstacleTrackerOptions & options);

  /// Runs one frame. `timestamp_s` is the measurement time of the frame, in
  /// seconds. A non-finite timestamp, or one earlier than the previous frame,
  /// resets the tracker before the measurements are processed.
  ObstacleTrackerUpdate update(
    const std::vector<ObstacleMeasurement> & measurements,
    double timestamp_s);

  /// Drops every track and forgets the timestamp history.
  void reset();

  std::size_t active_track_count() const;

  /// Options actually in use, after the constructor clamped them.
  const ObstacleTrackerOptions & options() const;

private:
  struct Track
  {
    std::uint64_t id{0U};
    double x_m{0.0};
    double y_m{0.0};
    double radius_m{0.0};
    std::size_t observation_count{0U};
    double last_observed_s{0.0};
    bool observed_this_frame{false};
    std::size_t measurement_index{0U};
  };

  ObstacleTrackerOptions options_;
  std::vector<Track> tracks_;
  std::uint64_t next_track_id_{1U};
  double last_timestamp_s_{0.0};
  bool has_timestamp_{false};
};

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__OBSTACLE_TRACKER_HPP_
