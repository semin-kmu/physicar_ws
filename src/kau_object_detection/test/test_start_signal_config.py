# Copyright 2026 KAU AMET Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Static checks of config/start_signal_detector.yaml."""

# Added 2026-08-25 alongside the sliding-window confirmation change.
#
# The point of that change was to alter the TEMPORAL policy and nothing else,
# so that its effect can be attributed to the window alone. These tests pin the
# spatial gates and the interface so a later edit cannot quietly widen the
# scope, and pin the new policy values so the window cannot drift back.
#
# Nothing here touches LiDAR object detection: this file reads one YAML.

import pathlib

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
CONFIG = PACKAGE_ROOT / 'config' / 'start_signal_detector.yaml'


def _parameters():
    with CONFIG.open() as handle:
        document = yaml.safe_load(handle)
    return document['start_signal_detector']['ros__parameters']


def test_yaml_parses_and_targets_the_start_signal_node():
    with CONFIG.open() as handle:
        document = yaml.safe_load(handle)
    assert list(document) == ['start_signal_detector']
    assert 'ros__parameters' in document['start_signal_detector']


def test_region_of_interest_is_unchanged():
    # Measured on the real PhysiCar camera. The sliding-window change did not
    # touch these and neither should anything that is not a deliberate
    # traffic-light hardening task.
    parameters = _parameters()
    assert parameters['roi_x_min'] == 280
    assert parameters['roi_x_max'] == 450
    assert parameters['roi_y_min'] == 135
    assert parameters['roi_y_max'] == 265


def test_colour_and_shape_gates_are_unchanged():
    parameters = _parameters()
    assert parameters['hue_min'] == 45
    assert parameters['hue_max'] == 85
    assert parameters['saturation_min'] == 150
    assert parameters['value_min'] == 150
    assert parameters['min_area_px'] == 60
    assert parameters['max_area_px'] == 5000
    assert parameters['min_aspect_ratio'] == 0.60
    assert parameters['max_aspect_ratio'] == 1.60
    assert parameters['min_fill_ratio'] == 0.60


def test_interface_and_timing_are_unchanged():
    parameters = _parameters()
    assert parameters['input_topic'] == '/camera/image_raw/compressed'
    assert parameters['output_topic'] == '/perception/start_permission'
    assert parameters['publish_period_s'] == 0.1
    assert parameters['camera_timeout_s'] == 0.5


def test_sliding_window_policy_is_shipped():
    parameters = _parameters()
    assert parameters['green_window_frames'] == 8
    assert parameters['green_required_frames'] == 4


def test_threshold_is_reachable():
    # A threshold above the window can never be met, which would forbid the
    # start for the whole race with no other symptom. The node refuses to start
    # on such a configuration; this catches it before that.
    parameters = _parameters()
    assert parameters['green_required_frames'] >= 1
    assert parameters['green_window_frames'] >= 1
    assert (parameters['green_required_frames'] <=
            parameters['green_window_frames'])


def test_consecutive_run_parameter_is_gone():
    # The node no longer declares it, so a leftover would be silently ignored
    # while reading as if the old five-in-a-row policy were still configured.
    assert 'green_confirmation_frames' not in _parameters()


def test_use_sim_time_is_not_pinned():
    # Pinning it would make it impossible to turn off on the real vehicle,
    # where there is no /clock and now() would stay at 0.
    assert 'use_sim_time' not in _parameters()
