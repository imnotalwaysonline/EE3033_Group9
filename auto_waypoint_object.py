# Setp 1: read the saved SLAM map from ROS2 /map
# Step 2: Automatically generating waypoint coordinates from the map
# Step 3: Filtering Unsafe Waypoints
# Step 4: Display Automatic Waypoints in RViz2


# Step 1
import rclpy # allow pyhton to communicate with ROS2
from rclpy.node import Node # used to create a ROS 2 node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy 
# QoSProfile - Configures how ROS 2 messages are delivered
# DurabilityPolicy - Determines whether late subscribers can receive previously published messages
# ReliabilityPolicy - Controls message delivery reliability
from nav_msgs.msg import OccupancyGrid # ROS 2 message type used to represent a 2D map

# Step 2
import math

# Step 4
from visualization_msgs.msg import Marker, MarkerArray

from collections import deque
from geometry_msgs.msg import PoseWithCovarianceStamped

# Step 6
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time
from copy import deepcopy
from std_msgs.msg import Bool

# Step 8: Automatic navigation
from nav2_msgs.action import NavigateToPose, Spin
from builtin_interfaces.msg import Duration

from rclpy.executors import ExternalShutdownException

# create ROS2 node
class AutoWaypointGenerator(Node):

    def __init__(self):
        super().__init__('auto_waypoint_return_home') # initializes the node with the name

        # Subscribe to the saved map published by Nav2
        map_qos = QoSProfile(depth=1) # Keep up to one message in the subscriber's history queue.
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Step 5: Receive robot position from AMCL
        self.robot_pose = None

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Step 4: Publisher for RViz waypoint marker
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/auto_waypoints', # new topic that need to sub
            marker_qos
        )

        # Step 6: Nav2 path validation client
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        # Step 8: Navigation action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Nav2 Spin action rotates the robot in place while the forward camera scans.
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.scanning = False
        self.spin_goal_handle = None
        self.spin_cancel_pending = False
        self.spin_steps_remaining = 0
        self.scan_pause_until = 0.0
        self.last_scan_position = None
        self.scan_spacing = 1.0  # original version: metres between full scans
        self.scan_pause_seconds = 0.7
        self.scan_timer = self.create_timer(0.1, self.scan_timer_callback)

        # Safety switch: Keep navigation disabled initially
        self.enable_navigation = True  # Dry run first; change only after verification

        # Step 10: Mission state and home pose
        self.home_pose = None
        self.mission_state = 'PATROL'  # PATROL, RETURN_PENDING, RETURNING, DONE
        self.search_round = 0  # No fixed maximum
        self.visited_waypoints = []
        # Failed navigation goals are retried after a cooldown in search rounds.
        self.failed_waypoints = {}  # (x, y) -> {'count': int, 'retry_round': int}
        self.failure_cooldown_rounds = 2
        self.max_failure_cooldown_rounds = 8
        self.scanned_waypoints = []
        self.visit_tolerance = 0.12  # metres; treat nearby points as the same
        self.scan_tolerance = 0.35  # metres; avoid repeated scans nearby
        self.revisit_mode = False
        self.round_scan_targets = []
        self.retry_search_timer = self.create_timer(2.0, self.retry_search_if_needed)
        self.next_search_allowed_at = 0.0
        self.object_triggered = False
        self.object_sub = self.create_subscription(
            Bool, '/object_detected', self.object_callback, 10
        )

        # Navigation state
        self.navigation_running = False
        self.navigation_goal_handle = None

        # Step 8C: Multiple-waypoint navigation
        self.current_waypoint_index = 0
        self.navigation_completed = False

        # Step 8B: Navigation safety settings
        self.navigation_timeout = 60.0  # overall goal timeout (existing setting)
        # Detect a goal that is not making progress, separately from overall timeout.
        self.progress_timeout = 20.0  # seconds without meaningful improvement
        self.progress_distance = 0.12  # metres of improvement needed to reset timer
        self.progress_goal_tolerance = 0.20  # do not flag final alignment as stuck
        self.active_goal_xy = None
        self.best_goal_distance = None
        self.last_progress_at = None
        self.last_pose_at = None
        self.navigation_started_at = None
        self.navigation_cancel_requested = False
        self.cancel_in_progress = False

        self.navigation_timer = self.create_timer(
            0.5,
            self.check_navigation_timeout
)

        # Step 6D: Validation state
        self.validated_waypoints = []
        self.validation_index = 0
        self.validation_running = False
        self.validation_waiting = False
        self.validation_started_at = 0.0
        self.validation_request_id = 0
        self.validation_active_id = None
        self.validation_goal_handle = None
        self.validation_timeout_seconds = 10.0

        # Check planner availability and request timeouts
        self.validation_timer = self.create_timer(
            0.5,
            self.validation_timer_callback
        )

        self._safe_cells_cache = None
        self.map_received = False
        self.saved_map = None
        self.waypoints_generated = False

        self.get_logger().info(
            'Automatic Waypoint Generator started. Waiting for map...'
        )

        # Step 5: Store robot position
    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.last_pose_at = time.monotonic()
        # Distance-to-goal improvement is more useful than raw motion:
        # oscillating in place should not reset the stuck timer.
        if self.navigation_running and self.active_goal_xy is not None:
            distance = math.hypot(
                self.robot_pose.position.x - self.active_goal_xy[0],
                self.robot_pose.position.y - self.active_goal_xy[1])
            if (self.best_goal_distance is None or
                    self.best_goal_distance - distance >= self.progress_distance):
                self.best_goal_distance = distance
                self.last_progress_at = time.monotonic()
        if self.home_pose is None:
            self.home_pose = deepcopy(msg.pose.pose)
            self.get_logger().info(
                f'Home saved: x={self.home_pose.position.x:.2f}, '
                f'y={self.home_pose.position.y:.2f}'
            )
        self.try_generate_waypoints()


    def try_generate_waypoints(self):

            if self.saved_map is None:
                return

            if self.robot_pose is None:
                self.get_logger().info('Waiting for AMCL pose...')
                return

            if self.waypoints_generated:
                return

            msg = self.saved_map

            # Convert robot position to grid coordinates
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y

            start_row, start_col = self.world_to_grid(
                msg, robot_x, robot_y
            )

            # Find connected free-space region
            reachable_cells = self.find_reachable_cells(
                msg, start_row, start_col
            )

            if not reachable_cells:
                self.get_logger().warn(
                    'No reachable safe cells found from starting pose. Check map and AMCL pose.'
                )
                return

            # Generate only waypoints in connected region
            candidates = self.generate_waypoints(msg, reachable_cells)
            new_points = [p for p in candidates
                          if not self.near_any(p, self.visited_waypoints,
                                              self.visit_tolerance)
                          and self.waypoint_retry_allowed(p)]
            if new_points:
                self.revisit_mode = False
                self.waypoints = new_points
            else:
                # Finite map exhausted: revisit reachable positions, but
                # prioritise viewpoints not yet scanned at this location.
                self.revisit_mode = True
                self.waypoints = [p for p in candidates if self.waypoint_retry_allowed(p)]
            self.get_logger().info(
                f'Search mode: {"revisit" if self.revisit_mode else "new positions"}; '
                f'{len(self.waypoints)} candidates.')

            self.publish_waypoint_markers(self.waypoints, ordered=False)

            self.get_logger().info(
                f'Generated {len(self.waypoints)} reachable waypoint candidates.'
            )

            self.waypoints_generated = True

            # Step 6D: Validate all waypoint candidates
            self.start_waypoint_validation()


    # Step 6D: Begin validating all waypoint candidates
    def start_waypoint_validation(self):

        self.validated_waypoints = []
        self.validation_index = 0
        self.validation_running = True
        self.validation_waiting = False

        self.get_logger().info(
            f'Starting validation of {len(self.waypoints)} waypoints.'
        )

        if not self.waypoints:
            self.finish_waypoint_validation()
            return

        self.validate_next_waypoint()

    def validation_timer_callback(self):
        if not self.validation_running:
            return
        if self.validation_waiting:
            if time.monotonic() - self.validation_started_at > self.validation_timeout_seconds:
                self.get_logger().warn(
                    f'Path validation timed out at waypoint {self.validation_index + 1}; skipping it.')
                old_handle = self.validation_goal_handle
                # Invalidate callbacks before requesting cancellation. A late response
                # from the old request must never validate the next waypoint.
                self.advance_validation()
                if old_handle is not None:
                    try:
                        old_handle.cancel_goal_async()
                    except Exception as exc:
                        self.get_logger().warn(f'Planner cancellation failed: {exc}')
            return
        self.validate_next_waypoint()

    def validate_next_waypoint(self):
        if not self.validation_running or self.validation_waiting:
            return
        if self.validation_index >= len(self.waypoints):
            self.finish_waypoint_validation()
            return
        if not self.path_client.server_is_ready():
            self.get_logger().warn('Waiting for Nav2 planner action server...')
            return

        index = self.validation_index
        x, y = self.waypoints[index]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = goal_pose.header.stamp
        start_pose.pose = deepcopy(self.robot_pose)
        request = ComputePathToPose.Goal()
        request.start = start_pose
        request.goal = goal_pose
        request.use_start = True
        request.planner_id = 'GridBased'

        self.validation_request_id += 1
        request_id = self.validation_request_id
        self.validation_active_id = request_id
        self.validation_goal_handle = None
        self.validation_waiting = True
        self.validation_started_at = time.monotonic()
        self.get_logger().info(
            f'Checking waypoint {index + 1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})')
        try:
            future = self.path_client.send_goal_async(request)
            future.add_done_callback(
                lambda f, rid=request_id, idx=index: self.path_goal_response(f, rid, idx))
        except Exception as exc:
            self.get_logger().error(f'Failed to send path request: {exc}')
            self.advance_validation()

    def path_goal_response(self, future, request_id, index):
        try:
            handle = future.result()
        except Exception as exc:
            if self.validation_active_id == request_id:
                self.get_logger().error(f'Path goal response failed: {exc}')
                self.advance_validation()
            return
        if not self.validation_running or self.validation_active_id != request_id:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn(f'Waypoint {index + 1}: planner rejected request.')
            self.advance_validation()
            return
        self.validation_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, rid=request_id, idx=index: self.path_result_callback(f, rid, idx))

    def path_result_callback(self, future, request_id, index):
        if not self.validation_running or self.validation_active_id != request_id:
            return
        try:
            response = future.result()
            valid = (response.status == GoalStatus.STATUS_SUCCEEDED
                     and len(response.result.path.poses) > 0)
        except Exception as exc:
            self.get_logger().warn(f'Path result failed for waypoint {index + 1}: {exc}')
            valid = False
        if valid:
            self.validated_waypoints.append(self.waypoints[index])
            self.get_logger().info(f'Waypoint {index + 1} VALID!')
        else:
            self.get_logger().warn(f'Waypoint {index + 1} invalid or unverified; skipping.')
        self.advance_validation()

    def advance_validation(self):
        self.validation_active_id = None
        self.validation_goal_handle = None
        self.validation_waiting = False
        self.validation_index += 1

    def finish_waypoint_validation(self):

        self.validation_running = False
        self.validation_waiting = False
        self.validation_active_id = None
        self.validation_goal_handle = None

        self.get_logger().info(
            f'Validation complete: '
            f'{len(self.validated_waypoints)}/{len(self.waypoints)} '
            'waypoints have valid Nav2 paths.'
        )

        # Step 7: Arrange validated waypoints
        self.ordered_waypoints = self.order_waypoints(
            self.validated_waypoints
        )

        self.get_logger().info(
            f'Ordered {len(self.ordered_waypoints)} waypoints.'
        )

        for i, (x, y) in enumerate(self.ordered_waypoints):
            self.get_logger().info(
                f'Route {i + 1}: x={x:.2f}, y={y:.2f}'
            )

        # Display ordered waypoints in RViz2
        self.publish_waypoint_markers(self.ordered_waypoints, ordered=True)

        # Prefer scans at viewpoints not previously scanned. When all have
        # been scanned, allow rescanning, since the object may have been missed.
        self.round_scan_targets = [p for p in self.ordered_waypoints
                                   if not self.near_any(p, self.scanned_waypoints,
                                                       self.scan_tolerance)]
        if not self.round_scan_targets:
            self.round_scan_targets = self.ordered_waypoints[::max(1, round(
                self.scan_spacing / 0.3))]
        # Only start patrol if the object has not been detected.
        if self.mission_state == 'RETURN_PENDING':
            self.start_return_home()
        elif self.mission_state == 'PATROL':
            if self.ordered_waypoints:
                self.start_navigation()
            else:
                self.get_logger().warn('No valid waypoints this round; retrying search.')
                self.begin_next_search_round()


    # Step 7: Structured sweeping order with obstacle-aware strip selection.
    # Uses the SAME validated points; no changes to map safety or coverage repair.
    def order_waypoints(self, waypoints):
        if not waypoints:
            return []
        # Organise points in map-aligned strips. Keep distinct stretches of
        # a row separate when there is a large gap (often a wall/obstacle).
        spacing = 0.30
        strips = {}
        for x, y in waypoints:
            row = round(y / spacing)
            strips.setdefault(row, []).append((x, y))
        runs = []
        for row in sorted(strips):
            line = sorted(strips[row], key=lambda p: p[0])
            run = []
            for point in line:
                if run and point[0] - run[-1][0] > 0.55:
                    runs.append(run)
                    run = []
                run.append(point)
            if run:
                runs.append(run)
        # Select the closest *reachable* strip endpoint by safe-grid travel
        # distance, not just Euclidean distance through walls.
        ordered = []
        current = (self.robot_pose.position.x, self.robot_pose.position.y)
        while runs:
            distances = self.safe_grid_distances(current)
            choices = []
            for i, run in enumerate(runs):
                for reverse, endpoint in ((False, run[0]), (True, run[-1])):
                    cell = self.world_to_grid(self.saved_map, *endpoint)
                    cost = distances.get(cell, float('inf'))
                    choices.append((cost, i, reverse))
            cost, idx, reverse = min(choices)
            if not math.isfinite(cost):
                # Still retain points: Nav2 will validate actual transitions.
                idx, reverse = min(
                    ((i, rev) for i, run in enumerate(runs) for rev in (False, True)),
                    key=lambda item: math.hypot(
                        runs[item[0]][-1 if item[1] else 0][0] - current[0],
                        runs[item[0]][-1 if item[1] else 0][1] - current[1]))
            run = runs.pop(idx)
            if reverse:
                run.reverse()
            ordered.extend(run)
            current = run[-1]
        return ordered

    def safe_grid_distances(self, start_xy):
        # Four-connected shortest-path distances on cells passing the same
        # clearance test as waypoint generation. Used for strip ordering only.
        msg = self.saved_map
        if msg is None:
            return {}
        width, height = msg.info.width, msg.info.height
        if self._safe_cells_cache is None:
            self._safe_cells_cache = {
                (row, col) for row in range(height) for col in range(width)
                if msg.data[row * width + col] == 0
                and self.is_safe_waypoint(msg, row, col)
            }
        safe = self._safe_cells_cache
        start = self.world_to_grid(msg, *start_xy)
        if not (0 <= start[0] < height and 0 <= start[1] < width):
            return {}
        # Robot may stand on a free cell just outside waypoint-clear space.
        # Find the nearest safe seed without crossing unknown/occupied cells.
        q = deque([start])
        seen = {start}
        seed = None
        while q:
            row, col = q.popleft()
            if (row, col) in safe:
                seed = (row, col)
                break
            if len(seen) > 500:
                break
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = row + dr, col + dc
                if (0 <= nr < height and 0 <= nc < width
                        and (nr, nc) not in seen
                        and msg.data[nr * width + nc] == 0):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        if seed is None:
            return {}
        q = deque([seed])
        distances = {seed: 0}
        while q:
            row, col = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = row + dr, col + dc
                if ((nr, nc) not in distances and 0 <= nr < height
                        and 0 <= nc < width and (nr, nc) in safe):
                    distances[(nr, nc)] = distances[(row, col)] + 1
                    q.append((nr, nc))
        return distances

    # Step 10: Object signal is latched once per mission.
    def object_callback(self, msg):
        if not msg.data or self.object_triggered or self.mission_state != 'PATROL':
            return
        self.object_triggered = True
        self.mission_state = 'RETURN_PENDING'
        self.get_logger().warn('Object confirmed. Stopping patrol; returning home.')
        if not self.enable_navigation:
            self.get_logger().info('Dry run: return-home navigation disabled.')
            return
        if self.scanning:
            self.spin_cancel_pending = True
            if self.spin_goal_handle is not None:
                self.spin_goal_handle.cancel_goal_async()
            # Spin result callback starts the return-home goal.
            return
        if self.navigation_running:
            self.navigation_cancel_requested = True
            if self.navigation_goal_handle is not None:
                self.cancel_navigation_goal()
            # Otherwise goal-response callback will cancel once accepted.
        else:
            self.start_return_home()

    def start_return_home(self):
        if self.mission_state not in ('PATROL', 'RETURN_PENDING'):
            return
        self.mission_state = 'RETURN_PENDING'
        if self.home_pose is None:
            self.get_logger().error('No home pose available; cannot return.')
            self.mission_state = 'DONE'
            return
        if not self.enable_navigation:
            self.get_logger().info('Dry run: home goal not sent.')
            return
        if self.navigation_running or self.scanning:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().error('Nav2 unavailable; return home not started.')
            self.mission_state = 'DONE'
            return
        self.mission_state = 'RETURNING'
        self.get_logger().info('Sending return-home goal.')
        self.send_navigation_goal(
            self.home_pose.position.x, self.home_pose.position.y,
            orientation=self.home_pose.orientation
        )

    @staticmethod
    def near_any(point, positions, tolerance):
        return any(math.hypot(point[0] - p[0], point[1] - p[1]) < tolerance
                   for p in positions)

    def waypoint_retry_allowed(self, point):
        record = self.failed_waypoints.get(point)
        return record is None or self.search_round >= record['retry_round']

    def record_failed_waypoint(self, point):
        record = self.failed_waypoints.get(point, {'count': 0, 'retry_round': 0})
        count = record['count'] + 1
        cooldown = min(self.max_failure_cooldown_rounds,
                       self.failure_cooldown_rounds * (2 ** min(count - 1, 10)))
        self.failed_waypoints[point] = {
            'count': count, 'retry_round': self.search_round + cooldown}
        self.get_logger().warn(
            f'Waypoint {point} failed {count} time(s); retry in search round '
            f'{self.search_round + cooldown + 1}.')

    def begin_next_search_round(self):
        # Called only after all patrol waypoints have been visited.
        if self.mission_state != 'PATROL' or self.object_triggered:
            return
        self.search_round += 1
        self.last_scan_position = None
        self.get_logger().warn(
            f'Object not found. Starting search round {self.search_round + 1}.')
        self.waypoints_generated = False
        self.navigation_completed = False
        # Defer generation to timer to avoid recursive loops if no paths validate.
        self.next_search_allowed_at = time.monotonic() + 2.0

    def retry_search_if_needed(self):
        # Retry if AMCL, Nav2 or valid waypoints were temporarily unavailable.
        if (time.monotonic() < self.next_search_allowed_at
                or self.mission_state != 'PATROL' or self.object_triggered
                or self.navigation_running or self.scanning or self.validation_running
                or self.validation_waiting):
            return
        if not self.waypoints_generated:
            self.try_generate_waypoints()
        elif (hasattr(self, 'ordered_waypoints')
              and self.ordered_waypoints
              and not self.navigation_completed
              and self.nav_client.server_is_ready()):
            self.navigate_to_next_waypoint()

    # Step 8C: Start automatic waypoint navigation
    def start_navigation(self):

        if self.mission_state != 'PATROL':
            return
        if not self.ordered_waypoints:
            self.get_logger().warn(
                'No validated waypoints available.'
            )
            return

        self.current_waypoint_index = 0
        self.navigation_completed = False

        self.get_logger().info(
            f'Prepared {len(self.ordered_waypoints)} waypoints for navigation.'
        )

        if not self.enable_navigation:
            self.get_logger().info(
                'Navigation disabled. Robot will not move.'
            )
            return

        self.navigate_to_next_waypoint()

    # Step 8C: Navigate to the next waypoint
    def navigate_to_next_waypoint(self):

        if not self.enable_navigation or self.mission_state != 'PATROL':
            return

        if self.navigation_running or self.scanning or self.navigation_completed:
            return

        if self.current_waypoint_index >= len(self.ordered_waypoints):

            self.navigation_completed = True

            self.get_logger().info(
                'All ordered waypoints completed successfully!'
            )
            self.begin_next_search_round()
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'NavigateToPose action server is not ready. '
                'Navigation stopped.'
            )
            return

        x, y = self.ordered_waypoints[self.current_waypoint_index]

        self.get_logger().info(
            f'Navigating to waypoint '
            f'{self.current_waypoint_index + 1}/'
            f'{len(self.ordered_waypoints)}: '
            f'x={x:.2f}, y={y:.2f}'
        )

        self.send_navigation_goal(x, y)

    # Step 8B: Monitor both overall timeout and lack of progress.
    def check_navigation_timeout(self):
        if not self.navigation_running or self.navigation_started_at is None:
            return
        if self.navigation_cancel_requested:
            return
        now = time.monotonic()
        elapsed = now - self.navigation_started_at
        reason = None
        if elapsed >= self.navigation_timeout:
            reason = 'Overall navigation timeout'
        elif (self.mission_state == 'PATROL' and self.active_goal_xy is not None
              and self.robot_pose is not None and self.last_progress_at is not None
              and self.best_goal_distance is not None
              and self.best_goal_distance > self.progress_goal_tolerance
              and self.last_pose_at is not None
              and now - self.last_pose_at < 3.0
              and now - self.last_progress_at >= self.progress_timeout):
            reason = 'No progress toward waypoint for 20 seconds'
        if reason is None:
            return
        self.navigation_cancel_requested = True
        self.get_logger().warn(f'{reason}; requesting goal cancellation.')
        if self.navigation_goal_handle is not None:
            self.cancel_navigation_goal()
        else:
            self.get_logger().warn('Waiting for goal acceptance before cancellation.')


    def cancel_navigation_goal(self):

        if self.navigation_goal_handle is None:
            return

        if self.cancel_in_progress:
            return

        self.cancel_in_progress = True

        self.get_logger().warn(
            'Sending navigation cancellation request.'
        )

        try:
            future = self.navigation_goal_handle.cancel_goal_async()
            future.add_done_callback(
                self.navigation_cancel_response
            )

        except Exception as exc:
            self.cancel_in_progress = False
            self.get_logger().error(
                f'Could not request cancellation: {exc}'
            )


    def navigation_cancel_response(self, future):
        self.cancel_in_progress = False

        try:
            response = future.result()

            if response.goals_canceling:
                self.get_logger().info(
                    'Nav2 accepted the cancellation request.'
                )
            else:
                self.get_logger().warn(
                    'Nav2 did not confirm goal cancellation.'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Navigation cancellation failed: {exc}'
            )


    # Step 8B: Send goal to Nav2
    def send_navigation_goal(self, x, y, orientation=None):

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        if orientation is None:
            goal.pose.pose.orientation.w = 1.0
        else:
            goal.pose.pose.orientation = deepcopy(orientation)

        self.navigation_running = True
        self.navigation_started_at = time.monotonic()
        self.active_goal_xy = (float(x), float(y))
        self.best_goal_distance = None
        self.last_progress_at = time.monotonic()
        self.navigation_cancel_requested = False
        self.cancel_in_progress = False
        self.navigation_goal_handle = None

        self.get_logger().info(
            f'Sending navigation goal: ({x:.2f}, {y:.2f})'
        )

        try:
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                self.navigation_goal_response
            )
        except Exception as exc:
            self.navigation_running = False
            self.get_logger().error(
                f'Failed to send navigation goal: {exc}'
            )

    # Step 8C: Check whether Nav2 accepted the goal
    def navigation_goal_response(self, future):

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.navigation_running = False
            self.get_logger().error(
                f'Navigation request failed: {exc}'
            )
            return

        if not goal_handle.accepted:
            self.navigation_running = False
            self.get_logger().warn(
                'Navigation goal rejected by Nav2.'
            )
            return

        self.navigation_goal_handle = goal_handle

        # If timeout occurred before goal acceptance,
        # request cancellation immediately.
        if self.navigation_cancel_requested or self.mission_state == 'RETURN_PENDING':
            self.navigation_cancel_requested = True
            self.cancel_navigation_goal()

        self.get_logger().info(
            'Navigation goal accepted by Nav2.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # Step 10: Handle patrol, cancellation and home results separately.
    def navigation_result_callback(self, future):
        self.navigation_running = False
        self.navigation_goal_handle = None
        self.navigation_started_at = None
        self.active_goal_xy = None
        self.best_goal_distance = None
        self.last_progress_at = None
        was_cancel_requested = self.navigation_cancel_requested
        self.navigation_cancel_requested = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Navigation result failed: {exc}')
            if self.mission_state == 'PATROL':
                self.record_failed_waypoint(self.ordered_waypoints[self.current_waypoint_index])
                self.current_waypoint_index += 1
                self.navigate_to_next_waypoint()
            else:
                self.mission_state = 'DONE'
            return

        if self.mission_state == 'RETURN_PENDING':
            # Wait for the previous Nav2 goal to finish before sending home.
            if response.status in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_ABORTED,
            ):
                self.start_return_home()
            else:
                self.get_logger().error(
                    f'Patrol goal ended with unexpected status {response.status}; '
                    'home goal not sent.'
                )
                self.mission_state = 'DONE'
            return

        if self.mission_state == 'RETURNING':
            self.mission_state = 'DONE'
            if response.status == GoalStatus.STATUS_SUCCEEDED and not was_cancel_requested:
                self.get_logger().info('HOME REACHED! Mission completed.')
            else:
                self.get_logger().error(
                    f'Return home did not succeed. Status: {response.status}'
                )
            return

        if self.mission_state != 'PATROL':
            return
        if response.status == GoalStatus.STATUS_SUCCEEDED and not was_cancel_requested:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached!'
            )
            point = self.ordered_waypoints[self.current_waypoint_index]
            self.current_waypoint_index += 1
            self.failed_waypoints.pop(point, None)
            if not self.near_any(point, self.visited_waypoints, self.visit_tolerance):
                self.visited_waypoints.append(point)
            if (point in self.round_scan_targets and
                    (self.last_scan_position is None or
                     math.hypot(point[0] - self.last_scan_position[0],
                                point[1] - self.last_scan_position[1]) >= self.scan_spacing)):
                self.last_scan_position = point
                self.active_scan_point = point
                self.start_scan()
            else:
                self.navigate_to_next_waypoint()
        else:
            self.get_logger().warn(
                f'Navigation stopped at waypoint {self.current_waypoint_index + 1}. '
                f'Action status: {response.status}'
            )
            # Skip unreachable/stalled waypoint and delay its next retry.
            self.record_failed_waypoint(self.ordered_waypoints[self.current_waypoint_index])
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()

    def start_scan(self):
        if self.mission_state != 'PATROL':
            return
        if not self.spin_client.server_is_ready():
            self.get_logger().warn('Nav2 /spin unavailable; skipping scan.')
            self.navigate_to_next_waypoint()
            return
        self.scanning = True
        self.spin_steps_remaining = 4  # Four 90-degree rotations
        self.scan_pause_until = time.monotonic() + self.scan_pause_seconds
        self.get_logger().info('Scanning 360 degrees in four 90-degree steps.')

    def scan_timer_callback(self):
        if not self.scanning or self.spin_goal_handle is not None:
            return
        if self.mission_state == 'RETURN_PENDING':
            self.scanning = False
            self.start_return_home()
            return
        if time.monotonic() < self.scan_pause_until:
            return
        if self.spin_steps_remaining == 0:
            if not self.near_any(self.active_scan_point, self.scanned_waypoints,
                                 self.scan_tolerance):
                self.scanned_waypoints.append(self.active_scan_point)
            self.scanning = False
            self.navigate_to_next_waypoint()
            return
        if not self.spin_client.server_is_ready():
            self.scanning = False
            self.navigate_to_next_waypoint()
            return
        goal = Spin.Goal()
        goal.target_yaw = math.pi / 2
        goal.time_allowance = Duration(sec=15)
        # Reserve handle state before sending, preventing timer duplicate sends.
        self.spin_goal_handle = 'pending'
        try:
            future = self.spin_client.send_goal_async(goal)
            future.add_done_callback(self.spin_goal_response)
        except Exception as exc:
            self.get_logger().warn(f'Spin request failed: {exc}')
            self.spin_goal_handle = None
            self.scanning = False
            self.navigate_to_next_waypoint()

    def spin_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Spin goal rejected')
            self.spin_goal_handle = handle
            if self.mission_state == 'RETURN_PENDING':
                handle.cancel_goal_async()
            result_future = handle.get_result_async()
            result_future.add_done_callback(self.spin_result_callback)
        except Exception as exc:
            self.get_logger().warn(f'Spin goal failed: {exc}')
            self.spin_goal_handle = None
            self.scanning = False
            if self.mission_state == 'RETURN_PENDING':
                self.start_return_home()
            else:
                self.navigate_to_next_waypoint()

    def spin_result_callback(self, future):
        self.spin_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'Spin result error: {exc}')
            status = None
        if self.mission_state == 'RETURN_PENDING':
            self.scanning = False
            self.start_return_home()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Spin unsuccessful ({status}); continuing patrol.')
            self.scanning = False
            self.navigate_to_next_waypoint()
            return
        self.spin_steps_remaining -= 1
        self.scan_pause_until = time.monotonic() + self.scan_pause_seconds

    # Step 8D: Request navigation cancellation
    def request_navigation_stop(self):

        self.enable_navigation = False
        self.navigation_completed = True

        if self.navigation_goal_handle is None:
            self.get_logger().warn(
                'No accepted navigation goal available to cancel.'
            )
            return None

        self.navigation_cancel_requested = True

        self.get_logger().warn(
            'Requesting cancellation before shutdown.'
        )

        return self.navigation_goal_handle.cancel_goal_async()

    def world_to_grid(self, msg, x, y):

        resolution = msg.info.resolution
        origin = msg.info.origin

        q = origin.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        dx = x - origin.position.x
        dy = y - origin.position.y

        # Inverse rotation into the map grid
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        col = math.floor(local_x / resolution)
        row = math.floor(local_y / resolution)

        return row, col

    def find_reachable_cells(self, msg, start_row, start_col):

        width = msg.info.width
        height = msg.info.height

        if not (0 <= start_row < height and
                0 <= start_col < width):
            return set()

        start_index = start_row * width + start_col

        # The starting pose need not coincide with a generated waypoint.
        # Require a known free starting cell, but do not apply the stricter
        # waypoint clearance requirement to that starting cell.
        if msg.data[start_index] != 0:
            self.get_logger().warn(
                'Starting pose is not in known free space; check the map and AMCL pose.')
            return set()

        seed_row, seed_col = start_row, start_col
        if not self.is_safe_waypoint(msg, start_row, start_col):
            # Search nearby connected FREE cells for a cell with waypoint clearance.
            # Never jump across an obstacle or an unknown cell to choose a seed.
            radius = math.ceil(0.60 / msg.info.resolution)
            search_queue = deque([(start_row, start_col)])
            searched = {start_index}
            seed = None
            while search_queue:
                row, col = search_queue.popleft()
                if self.is_safe_waypoint(msg, row, col):
                    seed = (row, col)
                    break
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = row + dr, col + dc
                    if not (0 <= nr < height and 0 <= nc < width):
                        continue
                    if math.hypot(nr - start_row, nc - start_col) > radius:
                        continue
                    index = nr * width + nc
                    if index in searched or msg.data[index] != 0:
                        continue
                    searched.add(index)
                    search_queue.append((nr, nc))
            if seed is None:
                self.get_logger().warn(
                    'No safe waypoint seed within 0.60 m of the starting pose.')
                return set()
            seed_row, seed_col = seed
            self.get_logger().info(
                'Starting cell lacks waypoint clearance; using nearby safe seed '
                f'({seed_row}, {seed_col}) for coverage generation.')

        seed_index = seed_row * width + seed_col
        visited = {seed_index}
        queue = deque([(seed_row, seed_col)])

        # Four-directional neighbours
        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1)
        ]

        while queue:

            row, col = queue.popleft()

            for dr, dc in directions:

                nr = row + dr
                nc = col + dc

                if not (0 <= nr < height and
                        0 <= nc < width):
                    continue

                index = nr * width + nc

                if index in visited:
                    continue

                if not self.is_safe_waypoint(msg, nr, nc):
                    continue

                visited.add(index)
                queue.append((nr, nc))

        return visited

    def map_callback(self, msg):

        if self.map_received:
            return

        self.map_received = True
        self.saved_map = msg
        self.try_generate_waypoints()

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        self.get_logger().info('Map received successfully!')
        self.get_logger().info(f'Map width: {width} cells')
        self.get_logger().info(f'Map height: {height} cells')
        self.get_logger().info(f'Map resolution: {resolution} m/cell')

        # Occupancy grid values:
        # 0 = free space
        # 100 = occupied
        # -1 = unknown

        free_cells = sum(1 for cell in msg.data if cell == 0)
        occupied_cells = sum(1 for cell in msg.data if cell == 100)
        unknown_cells = sum(1 for cell in msg.data if cell == -1)

        self.get_logger().info(f'Free cells: {free_cells}')
        self.get_logger().info(f'Occupied cells: {occupied_cells}')
        self.get_logger().info(f'Unknown cells: {unknown_cells}')

    
    # Step 2
    def generate_waypoints(self, msg, reachable_cells):
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height

        # Single fixed candidate grid; visited-waypoint tracking selects new positions.
        waypoint_spacing = 0.3  # metres

        # Convert spacing from metres to grid cells
        step = max(1, round(waypoint_spacing / resolution))

        # Map origin position and orientation
        origin = msg.info.origin
        origin_x = origin.position.x
        origin_y = origin.position.y

        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        waypoints = []

        # Generate one candidate grid only. No shifted grid patterns.
        for row in range(0, height, step):
            for col in range(0, width, step):
                index = row * width + col
                if index not in reachable_cells:
                    continue
                if msg.data[index] != 0:
                    continue
                if not self.is_safe_waypoint(msg, row, col):
                    continue

                local_x = (col + 0.5) * resolution
                local_y = (row + 0.5) * resolution
                x = origin_x + local_x * cos_yaw - local_y * sin_yaw
                y = origin_y + local_x * sin_yaw + local_y * cos_yaw
                waypoints.append((x, y))

        # Coverage repair: the fixed grid can miss narrow or offset safe regions.
        # Examine all reachable safe cells, grouped into small map-space bins,
        # and add a candidate only where no existing candidate is nearby.
        # This is NOT a shifted-grid round; it fills gaps in the same map.
        coverage_radius = 0.30  # metres; candidate centres within this distance count as covered
        bin_size = 0.20  # metres; group free cells to avoid excessive candidates
        spatial = {}

        def bucket(x, y, size):
            return (math.floor(x / size), math.floor(y / size))

        def register(point):
            key = bucket(point[0], point[1], coverage_radius)
            spatial.setdefault(key, []).append(point)

        def has_nearby(point):
            bx, by = bucket(point[0], point[1], coverage_radius)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for old in spatial.get((bx + dx, by + dy), []):
                        if math.hypot(point[0] - old[0], point[1] - old[1]) < coverage_radius:
                            return True
            return False

        for point in waypoints:
            register(point)

        # Pick one safe representative per bin. Avoid checking every cell against
        # every waypoint; the spatial index keeps the gap test inexpensive.
        representatives = {}
        for index in reachable_cells:
            row, col = divmod(index, width)
            if not self.is_safe_waypoint(msg, row, col):
                continue
            local_x = (col + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            x = origin_x + local_x * cos_yaw - local_y * sin_yaw
            y = origin_y + local_x * sin_yaw + local_y * cos_yaw
            key = bucket(x, y, bin_size)
            centre_x = (key[0] + 0.5) * bin_size
            centre_y = (key[1] + 0.5) * bin_size
            score = (x - centre_x) ** 2 + (y - centre_y) ** 2
            if key not in representatives or score < representatives[key][0]:
                representatives[key] = (score, (x, y))

        extra = 0
        for key in sorted(representatives):
            point = representatives[key][1]
            if not has_nearby(point):
                waypoints.append(point)
                register(point)
                extra += 1
        self.get_logger().info(
            f'Waypoint coverage repair added {extra} candidates in grid gaps.')
        return waypoints

    # Step 3: Check whether a candidate waypoint is safe
    def is_safe_waypoint(self, msg, row, col):

        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height

        # Minimum clearance from obstacles and unknown cells
        clearance = 0.15  # metres

        # Convert clearance to grid cells
        radius_cells = math.ceil(clearance / resolution)

        # Check cells around the candidate waypoint
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):

                # Only inspect cells within the clearance circle
                distance = math.hypot(dr, dc) * resolution

                if distance > clearance:
                    continue

                check_row = row + dr
                check_col = col + dc

                # Reject candidates too close to map boundaries
                if not (0 <= check_row < height and
                        0 <= check_col < width):
                    return False

                index = check_row * width + check_col

                # Reject obstacles, unknown cells and non-free cells
                if msg.data[index] != 0:
                    return False

        return True
    
    # Step 4: Publish waypoint markers for RViz2
    def publish_waypoint_markers(self, waypoints, ordered=False):
        # Show numbers and route only after Nav2 validation and route ordering.
        from geometry_msgs.msg import Point

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (x, y) in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'auto_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.10
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

            if ordered:
                number = Marker()
                number.header.frame_id = 'map'
                number.header.stamp = stamp
                number.ns = 'auto_waypoint_numbers'
                number.id = i
                number.type = Marker.TEXT_VIEW_FACING
                number.action = Marker.ADD
                number.pose.position.x = float(x)
                number.pose.position.y = float(y)
                number.pose.position.z = 0.24
                number.pose.orientation.w = 1.0
                number.scale.z = 0.16
                number.color.r = 1.0
                number.color.g = 1.0
                number.color.b = 1.0
                number.color.a = 1.0
                number.text = str(i + 1)
                marker_array.markers.append(number)

        if ordered and len(waypoints) > 1:
            route = Marker()
            route.header.frame_id = 'map'
            route.header.stamp = stamp
            route.ns = 'auto_waypoint_order_line'
            route.id = 0
            route.type = Marker.LINE_STRIP
            route.action = Marker.ADD
            route.pose.orientation.w = 1.0
            route.scale.x = 0.025
            route.color.r = 1.0
            route.color.g = 0.65
            route.color.b = 0.0
            route.color.a = 0.85
            for x, y in waypoints:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.035
                route.points.append(point)
            marker_array.markers.append(route)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f'Published {len(waypoints)} '
            f'{"numbered ordered" if ordered else "candidate"} waypoints to RViz2.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AutoWaypointGenerator()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()  

