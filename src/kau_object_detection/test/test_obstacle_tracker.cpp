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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection/obstacle_tracker.hpp"

namespace
{

using kau_object_detection::ObstacleMeasurement;
using kau_object_detection::ObstacleTracker;
using kau_object_detection::ObstacleTrackerOptions;

constexpr double kTolerance = 1e-9;

/// Base options for the tests. No obstacle count and no map position is assumed
/// anywhere; every test states the candidates it feeds in.
ObstacleTrackerOptions make_options(
  const double association_distance_m = 0.30,
  const double outlier_distance_m = 0.20,
  const double smoothing_alpha = 0.4,
  const std::size_t minimum_confirmation_frames = 2U,
  const double track_timeout_s = 0.7)
{
  ObstacleTrackerOptions options;
  options.association_distance_m = association_distance_m;
  options.outlier_distance_m = outlier_distance_m;
  options.smoothing_alpha = smoothing_alpha;
  options.minimum_confirmation_frames = minimum_confirmation_frames;
  options.track_timeout_s = track_timeout_s;
  return options;
}

ObstacleMeasurement make_measurement(
  const double x_m,
  const double y_m,
  const double radius_m = 0.09)
{
  ObstacleMeasurement measurement;
  measurement.x_m = x_m;
  measurement.y_m = y_m;
  measurement.radius_m = radius_m;
  return measurement;
}

TEST(ObstacleTrackerTest, FirstObservationStartsUnconfirmedTrack)
{
  ObstacleTracker tracker(make_options());

  const auto first = tracker.update({make_measurement(1.0, 2.0)}, 0.0);

  EXPECT_EQ(first.new_tracks, 1U);
  EXPECT_EQ(first.active_tracks, 1U);
  EXPECT_EQ(first.confirmed_observations, 1U);
  EXPECT_EQ(tracker.active_track_count(), 1U);
  // One observation is not enough with minimum_confirmation_frames = 2.
  EXPECT_TRUE(first.obstacles.empty());
}

TEST(ObstacleTrackerTest, TrackIsPublishedAfterTwoConsecutiveObservations)
{
  ObstacleTracker tracker(make_options());

  ASSERT_TRUE(tracker.update({make_measurement(1.0, 2.0)}, 0.0).obstacles.empty());
  const auto second = tracker.update({make_measurement(1.0, 2.0)}, 0.1);

  ASSERT_EQ(second.obstacles.size(), 1U);
  EXPECT_EQ(second.obstacles.front().observation_count, 2U);
  EXPECT_EQ(second.obstacles.front().measurement_index, 0U);
  EXPECT_EQ(second.new_tracks, 0U);
}

TEST(ObstacleTrackerTest, ThirdConfirmationFrameDelaysPublicationByOneMoreFrame)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 3U));

  EXPECT_TRUE(tracker.update({make_measurement(0.0, 0.0)}, 0.0).obstacles.empty());
  EXPECT_TRUE(tracker.update({make_measurement(0.0, 0.0)}, 0.1).obstacles.empty());
  EXPECT_EQ(tracker.update({make_measurement(0.0, 0.0)}, 0.2).obstacles.size(), 1U);
}

TEST(ObstacleTrackerTest, NewTrackIsInitialisedWithTheMeasurementItself)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  const auto first = tracker.update({make_measurement(1.0, -2.0, 0.09)}, 0.0);

  ASSERT_EQ(first.obstacles.size(), 1U);
  EXPECT_NEAR(first.obstacles.front().x_m, 1.0, kTolerance);
  EXPECT_NEAR(first.obstacles.front().y_m, -2.0, kTolerance);
  EXPECT_NEAR(first.obstacles.front().radius_m, 0.09, kTolerance);
}

TEST(ObstacleTrackerTest, ExponentialMovingAverageFollowsTheDocumentedFormula)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  ASSERT_EQ(tracker.update({make_measurement(1.0, 2.0)}, 0.0).obstacles.size(), 1U);
  // 0.112 m from the previous estimate, so the outlier gate leaves it alone.
  const auto second = tracker.update({make_measurement(1.10, 2.05)}, 0.1);

  ASSERT_EQ(second.obstacles.size(), 1U);
  // estimate = alpha * measurement + (1 - alpha) * previous_estimate
  EXPECT_NEAR(second.obstacles.front().x_m, (0.4 * 1.10) + (0.6 * 1.0), kTolerance);
  EXPECT_NEAR(second.obstacles.front().y_m, (0.4 * 2.05) + (0.6 * 2.0), kTolerance);
}

TEST(ObstacleTrackerTest, RadiusIsCarriedThroughWithoutSmoothing)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  ASSERT_EQ(tracker.update({make_measurement(0.0, 0.0, 0.09)}, 0.0).obstacles.size(), 1U);
  const auto second = tracker.update({make_measurement(0.01, 0.0, 0.12)}, 0.1);

  ASSERT_EQ(second.obstacles.size(), 1U);
  // The radius of the newest measurement is reported verbatim: no average of
  // 0.09 and 0.12 may appear here.
  EXPECT_NEAR(second.obstacles.front().radius_m, 0.12, kTolerance);
}

TEST(ObstacleTrackerTest, NeighbouringCandidatesKeepAOneToOneMatch)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  const auto first = tracker.update(
    {make_measurement(0.0, 0.0), make_measurement(0.25, 0.0)}, 0.0);
  ASSERT_EQ(first.obstacles.size(), 2U);
  const auto first_track_id = first.obstacles[0].track_id;
  const auto second_track_id = first.obstacles[1].track_id;
  ASSERT_NE(first_track_id, second_track_id);

  // Both measurements sit inside the association gate of both tracks, and the
  // input order is deliberately swapped. Closest-pair-first matching must still
  // link every track to the candidate that actually belongs to it.
  const auto second = tracker.update(
    {make_measurement(0.24, 0.0), make_measurement(0.02, 0.0)}, 0.1);

  ASSERT_EQ(second.obstacles.size(), 2U);
  EXPECT_EQ(second.active_tracks, 2U);
  EXPECT_EQ(second.new_tracks, 0U);

  EXPECT_EQ(second.obstacles[0].track_id, first_track_id);
  EXPECT_EQ(second.obstacles[0].measurement_index, 1U);
  EXPECT_NEAR(second.obstacles[0].x_m, (0.4 * 0.02) + (0.6 * 0.0), kTolerance);

  EXPECT_EQ(second.obstacles[1].track_id, second_track_id);
  EXPECT_EQ(second.obstacles[1].measurement_index, 0U);
  EXPECT_NEAR(second.obstacles[1].x_m, (0.4 * 0.24) + (0.6 * 0.25), kTolerance);
}

TEST(ObstacleTrackerTest, CandidateOutsideAssociationGateStartsItsOwnTrack)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  const auto first = tracker.update({make_measurement(0.0, 0.0)}, 0.0);
  ASSERT_EQ(first.obstacles.size(), 1U);
  const auto original_track_id = first.obstacles.front().track_id;

  const auto second = tracker.update({make_measurement(0.50, 0.0)}, 0.1);

  EXPECT_EQ(second.new_tracks, 1U);
  EXPECT_EQ(second.active_tracks, 2U);
  EXPECT_EQ(second.rejected_outliers, 0U);
  ASSERT_EQ(second.obstacles.size(), 1U);
  // The original track was not observed in this frame, so it must not appear.
  EXPECT_NE(second.obstacles.front().track_id, original_track_id);
  EXPECT_NEAR(second.obstacles.front().x_m, 0.50, kTolerance);
}

TEST(ObstacleTrackerTest, OutlierInsideAssociationGateDoesNotContaminateTheEstimate)
{
  ObstacleTracker tracker(make_options(0.30, 0.10, 0.5, 1U));

  ASSERT_EQ(tracker.update({make_measurement(1.0, 0.0)}, 0.0).obstacles.size(), 1U);

  // Linked to the same track, but 0.25 m away from the stable estimate.
  const auto jumped = tracker.update({make_measurement(1.25, 0.0)}, 0.1);
  EXPECT_EQ(jumped.rejected_outliers, 1U);
  EXPECT_EQ(jumped.confirmed_observations, 0U);
  EXPECT_EQ(jumped.new_tracks, 0U);
  EXPECT_EQ(jumped.active_tracks, 1U);
  // The track was not updated, so it was not observed and must not be reported.
  EXPECT_TRUE(jumped.obstacles.empty());

  const auto recovered = tracker.update({make_measurement(1.02, 0.0)}, 0.2);
  ASSERT_EQ(recovered.obstacles.size(), 1U);
  EXPECT_EQ(recovered.rejected_outliers, 0U);
  // Had the 1.25 m outlier been averaged in, the estimate could not be this
  // close to the pre-outlier value.
  EXPECT_NEAR(recovered.obstacles.front().x_m, (0.5 * 1.02) + (0.5 * 1.0), kTolerance);
}

TEST(ObstacleTrackerTest, TrackSurvivesShortDropoutButIsNeverRepublished)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U, 0.7));

  const auto first = tracker.update({make_measurement(3.0, 4.0)}, 0.0);
  ASSERT_EQ(first.obstacles.size(), 1U);
  const auto track_id = first.obstacles.front().track_id;

  const auto dropout = tracker.update({}, 0.1);
  EXPECT_EQ(dropout.active_tracks, 1U);
  EXPECT_EQ(dropout.timed_out_tracks, 0U);
  // The track is still alive internally, but no predicted position leaves the
  // tracker: an unobserved track is never an output candidate.
  EXPECT_TRUE(dropout.obstacles.empty());

  const auto reacquired = tracker.update({make_measurement(3.0, 4.0)}, 0.2);
  ASSERT_EQ(reacquired.obstacles.size(), 1U);
  EXPECT_EQ(reacquired.new_tracks, 0U);
  // Same identity: the dropout did not restart the confirmation.
  EXPECT_EQ(reacquired.obstacles.front().track_id, track_id);
  EXPECT_EQ(reacquired.obstacles.front().observation_count, 2U);
}

TEST(ObstacleTrackerTest, TrackIsDeletedAfterTheTimeout)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U, 0.7));

  const auto first = tracker.update({make_measurement(3.0, 4.0)}, 0.0);
  ASSERT_EQ(first.obstacles.size(), 1U);
  const auto track_id = first.obstacles.front().track_id;

  const auto expired = tracker.update({}, 0.8);
  EXPECT_EQ(expired.timed_out_tracks, 1U);
  EXPECT_EQ(expired.active_tracks, 0U);
  EXPECT_EQ(tracker.active_track_count(), 0U);

  // The same position after the timeout is a new obstacle as far as the tracker
  // is concerned, so it has to earn its confirmation again.
  const auto reappeared = tracker.update({make_measurement(3.0, 4.0)}, 0.9);
  ASSERT_EQ(reappeared.obstacles.size(), 1U);
  EXPECT_EQ(reappeared.new_tracks, 1U);
  EXPECT_NE(reappeared.obstacles.front().track_id, track_id);
  EXPECT_EQ(reappeared.obstacles.front().observation_count, 1U);
}

TEST(ObstacleTrackerTest, BackwardTimestampResetsTheTracker)
{
  ObstacleTracker tracker(make_options());

  ASSERT_TRUE(tracker.update({make_measurement(1.0, 1.0)}, 5.0).obstacles.empty());
  const auto confirmed = tracker.update({make_measurement(1.0, 1.0)}, 5.1);
  ASSERT_EQ(confirmed.obstacles.size(), 1U);

  const auto rewound = tracker.update({make_measurement(1.0, 1.0)}, 4.9);
  EXPECT_TRUE(rewound.reset_performed);
  EXPECT_EQ(rewound.new_tracks, 1U);
  EXPECT_EQ(rewound.active_tracks, 1U);
  // Everything before the rewind was discarded, so the rebuilt track has to be
  // confirmed again before it may be published.
  EXPECT_TRUE(rewound.obstacles.empty());
}

TEST(ObstacleTrackerTest, NonFiniteTimestampResetsAndProducesNothing)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  ASSERT_EQ(tracker.update({make_measurement(0.0, 0.0)}, 0.0).obstacles.size(), 1U);

  const auto broken = tracker.update(
    {make_measurement(0.0, 0.0)}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(broken.reset_performed);
  EXPECT_TRUE(broken.obstacles.empty());
  EXPECT_EQ(broken.active_tracks, 0U);
  EXPECT_EQ(tracker.active_track_count(), 0U);
}

TEST(ObstacleTrackerTest, NonFiniteMeasurementIsIgnored)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  const auto result = tracker.update(
    {
      make_measurement(std::numeric_limits<double>::infinity(), 0.0),
      make_measurement(1.0, 1.0),
    },
    0.0);

  ASSERT_EQ(result.obstacles.size(), 1U);
  EXPECT_EQ(result.active_tracks, 1U);
  EXPECT_EQ(result.obstacles.front().measurement_index, 1U);
}

TEST(ObstacleTrackerTest, ExplicitResetClearsEveryTrack)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  ASSERT_EQ(tracker.update({make_measurement(0.0, 0.0)}, 0.0).obstacles.size(), 1U);
  tracker.reset();
  EXPECT_EQ(tracker.active_track_count(), 0U);

  const auto after_reset = tracker.update({make_measurement(0.0, 0.0)}, 0.1);
  EXPECT_EQ(after_reset.new_tracks, 1U);
  EXPECT_FALSE(after_reset.reset_performed);
}

TEST(ObstacleTrackerTest, ArbitraryCandidateCountIsTrackedWithoutAFixedAssumption)
{
  ObstacleTracker tracker(make_options());

  // Eleven candidates on a wide ring, well outside each other's association
  // gate. The count is deliberately not the practice-map cone count: the
  // tracker must derive the number of tracks from the measurements alone.
  constexpr std::size_t kCandidateCount = 11U;
  std::vector<ObstacleMeasurement> measurements;
  measurements.reserve(kCandidateCount);
  for (std::size_t index = 0U; index < kCandidateCount; ++index) {
    const double angle = (2.0 * 3.14159265358979323846 *
      static_cast<double>(index)) / static_cast<double>(kCandidateCount);
    measurements.push_back(make_measurement(3.0 * std::cos(angle), 3.0 * std::sin(angle)));
  }

  const auto first = tracker.update(measurements, 0.0);
  EXPECT_EQ(first.new_tracks, kCandidateCount);
  EXPECT_TRUE(first.obstacles.empty());

  const auto second = tracker.update(measurements, 0.1);
  EXPECT_EQ(second.active_tracks, kCandidateCount);
  EXPECT_EQ(second.new_tracks, 0U);
  EXPECT_EQ(second.rejected_outliers, 0U);
  ASSERT_EQ(second.obstacles.size(), kCandidateCount);

  std::set<std::uint64_t> track_ids;
  std::set<std::size_t> measurement_indices;
  for (const auto & obstacle : second.obstacles) {
    track_ids.insert(obstacle.track_id);
    measurement_indices.insert(obstacle.measurement_index);
  }
  // One track per candidate and one candidate per track: no measurement was
  // consumed twice.
  EXPECT_EQ(track_ids.size(), kCandidateCount);
  EXPECT_EQ(measurement_indices.size(), kCandidateCount);
}

TEST(ObstacleTrackerTest, PartialObservationPublishesOnlyTheObservedTracks)
{
  ObstacleTracker tracker(make_options(0.30, 0.20, 0.4, 1U));

  const auto first = tracker.update(
    {make_measurement(0.0, 0.0), make_measurement(2.0, 0.0), make_measurement(4.0, 0.0)}, 0.0);
  ASSERT_EQ(first.obstacles.size(), 3U);
  const auto middle_track_id = first.obstacles[1].track_id;

  // Only the middle obstacle is measured in this frame.
  const auto second = tracker.update({make_measurement(2.0, 0.0)}, 0.1);

  EXPECT_EQ(second.active_tracks, 3U);
  ASSERT_EQ(second.obstacles.size(), 1U);
  EXPECT_EQ(second.obstacles.front().track_id, middle_track_id);
}

TEST(ObstacleTrackerTest, OptionsAreClampedIntoAUsableRange)
{
  ObstacleTrackerOptions options;
  options.smoothing_alpha = 1.7;
  options.minimum_confirmation_frames = 0U;
  options.association_distance_m = -1.0;
  options.track_timeout_s = std::numeric_limits<double>::quiet_NaN();

  const ObstacleTracker tracker(options);

  EXPECT_NEAR(tracker.options().smoothing_alpha, 1.0, kTolerance);
  EXPECT_EQ(tracker.options().minimum_confirmation_frames, 1U);
  EXPECT_NEAR(tracker.options().association_distance_m, 0.0, kTolerance);
  EXPECT_NEAR(tracker.options().track_timeout_s, 0.0, kTolerance);
}

}  // namespace
