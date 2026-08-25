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

"""Static checks of the shipped YAML, launch and build files."""

# These run without a ROS graph. They exist to catch the two regressions this
# package is most exposed to:
#
#   1. a map/tracker/ROI parameter creeping back into the YAML, and
#   2. a build- or runtime dependency on kau_object_detection creeping back in.
#
# Both would be silent at build time and only show up as wrong behaviour on the
# vehicle, so they are asserted here instead.

import ast
import pathlib

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent

# Every name that would drag a map frame, a tracker or a ROI back in. A YAML
# containing any of these is a bug: the node does not declare them, so the
# value would be silently ignored while reading as if it were configured.
FORBIDDEN_PARAMETERS = (
    'world_pose_topic',
    'world_pose_base_frame',
    'world_pose_timeout_s',
    'lidar_offset_x_m',
    'lidar_offset_y_m',
    'track_boundary_tolerance_m',
    'track_outer_x',
    'track_outer_y',
    'track_inner_x',
    'track_inner_y',
    'track_yaml_path',
    'object_list_pose_timeout_s',
    'raw_diagnostic_object_list_topic',
    'raw_diagnostic_object_list_enabled',
    'temporal_tracking_enabled',
    'association_distance_m',
    'outlier_distance_m',
    'smoothing_alpha',
    'minimum_confirmation_frames',
    'track_timeout_s',
)


def _config(name):
    return PACKAGE_ROOT / 'config' / name


def _launch_files():
    return sorted((PACKAGE_ROOT / 'launch').glob('*.launch.py'))


def _load(path):
    with path.open() as handle:
        return yaml.safe_load(handle)


def test_detector_yaml_parses_and_targets_the_right_node():
    document = _load(_config('lane_object_detector.yaml'))
    assert list(document) == ['lane_laser_object_detector']
    assert 'ros__parameters' in document['lane_laser_object_detector']


def test_start_signal_yaml_parses_and_targets_the_right_node():
    document = _load(_config('start_signal_detector.yaml'))
    assert list(document) == ['start_signal_detector']
    assert 'ros__parameters' in document['start_signal_detector']


def test_detector_yaml_publishes_in_base_link():
    parameters = _load(_config('lane_object_detector.yaml'))[
        'lane_laser_object_detector']['ros__parameters']
    # The Lane-only contract. A map frame here would defeat the whole package.
    assert parameters['object_list_frame_id'] == 'base_link'
    assert parameters['expected_frame_id'] == 'lidar_link'
    assert parameters['input_topic'] == '/scan_filtered'
    assert parameters['object_list_topic'] == '/perception/obstacles'


def test_detector_yaml_has_no_map_tracker_or_roi_parameter():
    parameters = _load(_config('lane_object_detector.yaml'))[
        'lane_laser_object_detector']['ros__parameters']
    present = [name for name in FORBIDDEN_PARAMETERS if name in parameters]
    assert not present, 'map/tracker/ROI parameters must not be here: %s' % present


def test_no_yaml_value_names_the_map_frame():
    for name in ('lane_object_detector.yaml', 'start_signal_detector.yaml'):
        document = _load(_config(name))
        for block in document.values():
            for key, value in block['ros__parameters'].items():
                if isinstance(value, str):
                    assert value != 'map', '%s: %s is the map frame' % (name, key)


def test_watchdog_publish_period_stays_below_the_scan_timeout():
    # The node throws on start-up otherwise, so a bad shipped default would
    # only be discovered on the vehicle.
    parameters = _load(_config('lane_object_detector.yaml'))[
        'lane_laser_object_detector']['ros__parameters']
    assert (parameters['object_list_watchdog_publish_period_s'] <
            parameters['object_list_scan_timeout_s'])


def test_use_sim_time_is_not_pinned_in_any_yaml():
    # Pinning it would make it impossible to turn off on the real vehicle,
    # where there is no /clock and now() would stay at 0.
    for name in ('lane_object_detector.yaml', 'start_signal_detector.yaml'):
        document = _load(_config(name))
        for block in document.values():
            assert 'use_sim_time' not in block['ros__parameters'], name


def test_start_signal_yaml_keeps_the_validated_spatial_values():
    # The SPATIAL detection gates are the values the source package was
    # validated with on 2026-08-25. The 2026-08-25 sliding-window change
    # deliberately touched none of them, so that the effect of that change can
    # be attributed to the temporal policy alone. Adjusting ROI, combining
    # RGB/HSV, black panel checks and dynamic ROI are all separate tasks;
    # changing a value here without doing one is what this test guards against.
    parameters = _load(_config('start_signal_detector.yaml'))[
        'start_signal_detector']['ros__parameters']
    assert parameters['roi_x_min'] == 280
    assert parameters['roi_x_max'] == 450
    assert parameters['roi_y_min'] == 135
    assert parameters['roi_y_max'] == 265
    assert parameters['hue_min'] == 45
    assert parameters['hue_max'] == 85
    assert parameters['saturation_min'] == 150
    assert parameters['value_min'] == 150
    assert parameters['min_area_px'] == 60
    assert parameters['max_area_px'] == 5000
    assert parameters['min_aspect_ratio'] == 0.6
    assert parameters['max_aspect_ratio'] == 1.6
    assert parameters['min_fill_ratio'] == 0.6
    # Interface and timing, also unchanged by that work.
    assert parameters['input_topic'] == '/camera/image_raw/compressed'
    assert parameters['output_topic'] == '/perception/start_permission'
    assert parameters['camera_timeout_s'] == 0.5
    assert parameters['publish_period_s'] == 0.1


def test_start_signal_yaml_ships_the_sliding_window_policy():
    parameters = _load(_config('start_signal_detector.yaml'))[
        'start_signal_detector']['ros__parameters']
    assert parameters['green_window_frames'] == 8
    assert parameters['green_required_frames'] == 4
    # An unreachable threshold would forbid the start for the whole race.
    assert (parameters['green_required_frames'] <=
            parameters['green_window_frames'])
    assert parameters['green_required_frames'] >= 1


def test_start_signal_yaml_has_no_consecutive_run_parameter_left():
    # The node does not declare it any more, so a leftover would be silently
    # ignored while reading as if the old policy were still configured.
    parameters = _load(_config('start_signal_detector.yaml'))[
        'start_signal_detector']['ros__parameters']
    assert 'green_confirmation_frames' not in parameters


def test_every_launch_file_is_valid_python():
    files = _launch_files()
    assert len(files) == 3
    for path in files:
        ast.parse(path.read_text(), filename=str(path))


def test_launch_files_declare_generate_launch_description():
    for path in _launch_files():
        tree = ast.parse(path.read_text(), filename=str(path))
        names = [node.name for node in ast.walk(tree)
                 if isinstance(node, ast.FunctionDef)]
        assert 'generate_launch_description' in names, path.name


def test_launch_files_only_reference_this_package():
    # A get_package_share_directory or a Node(package=...) pointing at
    # kau_object_detection would make this package depend on it at runtime.
    for path in _launch_files():
        text = path.read_text()
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith('#') or 'README' in stripped:
                continue
            assert 'kau_object_detection/' not in stripped, path.name
            assert "'kau_object_detection'" not in stripped, path.name
            assert '"kau_object_detection"' not in stripped, path.name


def test_launch_files_expose_use_sim_time():
    for path in _launch_files():
        assert 'use_sim_time' in path.read_text(), path.name


def _strip_cmake_comments(text):
    # Only directives matter here. The file explains in prose why it must never
    # gain a find_package on the source package, and that sentence is not a
    # dependency.
    lines = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        lines.append(line.split('#', 1)[0])
    return '\n'.join(lines)


def test_build_files_do_not_depend_on_the_source_package():
    cmake = _strip_cmake_comments((PACKAGE_ROOT / 'CMakeLists.txt').read_text())
    package_xml = (PACKAGE_ROOT / 'package.xml').read_text()

    assert 'find_package(kau_object_detection)' not in cmake
    assert 'find_package(kau_object_detection ' not in cmake
    assert 'kau_object_detection)' not in cmake
    # No target_link_libraries against the other package's libraries either.
    assert 'kau_object_detection ' not in cmake
    assert '<depend>kau_object_detection</depend>' not in package_xml
    assert '<build_depend>kau_object_detection</build_depend>' not in package_xml
    assert '<exec_depend>kau_object_detection</exec_depend>' not in package_xml


def test_no_source_file_includes_the_other_package():
    roots = (PACKAGE_ROOT / 'include', PACKAGE_ROOT / 'src', PACKAGE_ROOT / 'test')
    for root in roots:
        for path in list(root.rglob('*.hpp')) + list(root.rglob('*.cpp')):
            for line in path.read_text().splitlines():
                if not line.lstrip().startswith('#include'):
                    continue
                assert 'kau_object_detection/' not in line, '%s: %s' % (path.name, line)
