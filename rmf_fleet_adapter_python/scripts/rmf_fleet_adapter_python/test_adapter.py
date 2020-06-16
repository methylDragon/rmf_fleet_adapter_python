from rclpy.node import Node

from rmf_task_msgs.msg import TaskSummary
import rmf_dispenser_msgs.msg as dispenser_msgs

from rclpy.executors import SingleThreadedExecutor
import rclpy

import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter as adpt

from multiprocessing import Process
from functools import partial
import datetime
import random

###############################################################################
# PARAMS
###############################################################################

pickup_name = "pickup"
dropoff_name = "dropoff"

quiet_dispenser_name = "quiet"
flaky_dispenser_name = "flaky"

###############################################################################
# CONSTS
###############################################################################

DISPENSER_RESULT_ACKNOWLEDGED = 0
DISPENSER_RESULT_SUCCESS = 1
DISPENSER_RESULT_FAILED = 2

DISPENSER_STATE_IDLE = 0
DISPENSER_STATE_BUSY = 1
DISPENSER_STATE_OFFLINE = 2

TASK_STATE_QUEUED = 0
TASK_STATE_ACTIVE = 1
TASK_STATE_COMPLETED = 2
TASK_STATE_FAILED = 3


###############################################################################
# CLASSES
###############################################################################

class MockQuietDispenser(Node):
    """
    This mock dispenser will not publish any states; it will only publish a
    successful result and nothing else.
    """
    def __init__(self, name):
        super().__init__(name)

        # Variables
        self.name = name
        self.tasks = {}

        self.success_flag = False
        self._timer = None

        # Pub-sub
        self.result_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            'dispenser_results',
            10
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            'dispenser_requests',
            self._process_request_cb,
            10
        )

    def _process_request_cb(self, msg):
        print("QUIET RECEIVED A CB")
        # Process request if addressed to this dispenser
        if msg.target_guid != self.name:
            return

        if not self.tasks.get(msg.request_guid):
            self.tasks[msg.request_guid] = False
            status = DISPENSER_RESULT_ACKNOWLEDGED

            self.timer = self.create_timer(
                0.01,
                partial(self._timer_cb, msg=msg)
            )
        else:
            status = DISPENSER_RESULT_SUCCESS

        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = status
        result.source_guid = self.name
        result.request_guid = msg.request_guid

        self.result_pub.publish(result)

    def _timer_cb(self, msg):
        if not self.timer:
            return

        self.timer.reset()

        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = DISPENSER_RESULT_SUCCESS
        result.source_guid = self.name
        result.request_guid = msg.request_guid

        self.result_pub.publish(result)
        self.success_flag = True


class MockFlakyDispenser(Node):
    """
    This mock dispenser will not publish any results; it will only publish
    states. This is representative of network issues where a result might not
    actually arrive, but the state heartbeats can still get through.
    """
    class RequestEntry():
        def __init__(self, request, publish_count):
            self.request = request
            self.publish_count = publish_count

    def __init__(self, name):
        super().__init__(name)

        # Variables
        self.name = name
        self.request_queue = []

        self.success_flag = False
        self._fulfilled_flag = False

        # Pub-sub
        self.state_pub = self.create_publisher(
            dispenser_msgs.DispenserState,
            'dispenser_states',
            10
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            'dispenser_requests',
            self._process_request_cb,
            10
        )

        self._timer = self.create_timer(
            0.1,
            self._timer_cb
        )

    def _process_request_cb(self, msg):
        # Add requests to queue if addressed to this dispenser
        print("FLAKY REQUEST RECEIVED", msg)
        if msg.target_guid != self.name:
            return

        print("APPENDING REQUEST ENTRY")
        self.request_queue.append(self.RequestEntry(msg, 0))

    def _timer_cb(self):
        msg = dispenser_msgs.DispenserState()
        msg.guid = self.name

        if not self.request_queue:  # If empty
            msg.mode = DISPENSER_STATE_IDLE
        else:
            msg.mode = DISPENSER_STATE_BUSY

        msg.time = self.get_clock().now().to_msg()
        msg.seconds_remaining = 0.1

        # Increment publish_count of all requests in queue
        for req in self.request_queue:
            msg.request_guid_queue.append(req.request.request_guid)
            req.publish_count += 1

        initial_count = len(self.request_queue)

        # Remove all requests with publish count > 2
        self.request_queue = list(filter(lambda x: x.publish_count > 2,
                                  self.request_queue))

        # If any requests were removed, set flags to True
        if len(self.request_queue) < initial_count:
            if not self._fulfilled_flag:
                self._fulfilled_flag = True
                self.success_flag = True

        self.state_pub.publish(msg)


class MockRobotCommand(adpt.RobotCommandHandle):
    class EventListener(graph.lane.Executor):
        def __init__(self, dock_to_wp, wp):
            graph.lane.Executor.__init__(self)

            self.dock_to_wp = dock_to_wp
            self.wp = wp

        def dock_execute(self, dock):
            self.dock_to_wp[dock.dock_name] = self.wp
            print("DOCK EVENT EXECUTED FOR DOCK:", dock.dock_name)

        # And these are all overrided but meant to do nothing
        def door_open_execute(self, door_open):
            return

        def door_close_execute(self, door_close):
            return

        def lift_door_open_execute(self, lift_door_open):
            return

        def lift_door_close_execute(self, lift_door_close):
            return

        def lift_move_execute(self, lift_move):
            return

    def __init__(self, node, graph):
        adpt.RobotCommandHandle.__init__(self)

        self.updater = None

        self.active = False
        self.node = node
        self.timer = None
        self.current_waypoint_target = 0
        self.dockings = {}
        self.visited_waypoints = {}
        self.dock_to_wp = {}

        for i in range(graph.num_lanes):
            lane = graph.get_lane(i)

            # lane.entry and lane.exit are a Lane::Node wrappers
            if lane.entry.event:
                executor = self.EventListener(self.dock_to_wp,
                                              lane.exit.waypoint_index)
                # TODO(CH3): Why is the passed in event execute()
                # not implemented?!?
                try:
                    lane.entry.event.execute(executor)
                except Exception:
                    print(type(lane.entry.event))
                    print("EVENT EXECUTE FOR LANE", i, "NOT IMPLEMENTED")

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,  # function!
                        path_finished_callback):
        self.current_waypoint_target = 0
        self.active = True
        self.timer = self.node.create_timer(
            0.01,
            partial(self._timer_cb,
                    waypoints=waypoints,
                    next_arrival_estimator=next_arrival_estimator,
                    path_finished_callback=path_finished_callback)
        )

    def stop(self):
        self.timer.reset()

    def dock(self, dock_name, docking_finished_callback):
        assert dock_name in self.dock_to_wp

        # For both dockings and visited waypoints, increment the associated
        # keyed values by 1. Or start it off at 1 if it doesn't exist yet.
        self.dockings[dock_name] = self.dockings.get(dock_name, 0) + 1

        waypoint = self.dock_to_wp[dock_name]
        self.visited_waypoints[waypoint] = \
            self.visited_waypoints.get(waypoint, 0) + 1

        docking_finished_callback()
        print("DOCKING FINISHED")

    def _timer_cb(self,
                  waypoints,
                  next_arrival_estimator,
                  path_finished_callback):
        if not self.active:
            return

        if self.current_waypoint_target < len(waypoints):
            self.current_waypoint_target += 1

        if self.updater:
            # This waypoint is a plan waypoint, NOT graph waypoint!!
            previous_waypoint = waypoints[self.current_waypoint_target - 1]

            if previous_waypoint.graph_index.has_value:
                print("UPDATEHANDLE UPDATING ROBOT POSITION:",
                      previous_waypoint.graph_index.value)

                self.updater.update_position(
                    previous_waypoint.graph_index.value,
                    previous_waypoint.position[2]
                )
                self.visited_waypoints[previous_waypoint.graph_index.value] = (
                    self.visited_waypoints
                        .get(previous_waypoint.graph_index.value, 0)
                    + 1
                )
            else:
                print("UPDATEHANDLE UPDATING ROBOT POSITION DEFAULT:",
                      previous_waypoint.position)
                # TODO(CH3): NOTE(CH3): Confirm this magic string is wanted
                self.updater.update_position("test_map",
                                             previous_waypoint.position)

        if self.current_waypoint_target < len(waypoints):
            # Again, this waypoint is a plan waypoint! NOT a graph waypoint!!
            waypoint = waypoints[self.current_waypoint_target]
            test_delay = (datetime.timedelta(milliseconds=750)
                          * self.current_waypoint_target)

            delayed_arrival_time = waypoint.time + test_delay
            now = datetime.datetime.fromtimestamp(self.node
                                                  .get_clock()
                                                  .now()
                                                  .seconds_nanoseconds()[0])

            remaining_time = now - (now - delayed_arrival_time)
            next_arrival_estimator(self.current_waypoint_target,
                                   remaining_time)
        else:
            self.active = False
            self.timer.reset()
            path_finished_callback()
            print("PATH FINISHED")


if __name__ == "__main__":
    # INIT RCL ================================================================
    rclpy.init()
    adpt.init_rclcpp()

    # INIT GRAPH ==============================================================
    map_name = "test_map"
    test_graph = graph.Graph()

    test_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    test_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
    test_graph.add_waypoint(map_name, [5.0, -5.0]).set_holding_point(True)  # 2
    test_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    test_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
    test_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    test_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    test_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    test_graph.add_waypoint(map_name, [0.0, 5.0])  # 8
    test_graph.add_waypoint(map_name, [5.0, 5.0]).set_holding_point(True)  # 9
    test_graph.add_waypoint(map_name, [0.0, 10.0])  # 10

    assert test_graph.get_waypoint(2).holding_point
    assert test_graph.get_waypoint(9).holding_point
    assert not test_graph.get_waypoint(10).holding_point

    """
                     10(D)
                      |
                      |
                      8------9
                      |      |
                      |      |
        3------4------5------6------7(D)
                      |      |
                      |      |
                      1------2
                      |
                      |
                      0
   """

    test_graph.add_bidir_lane(0, 1)  # 0   1
    test_graph.add_bidir_lane(1, 2)  # 2   3
    test_graph.add_bidir_lane(1, 5)  # 4   5
    test_graph.add_bidir_lane(2, 6)  # 6   7
    test_graph.add_bidir_lane(3, 4)  # 8   9
    test_graph.add_bidir_lane(4, 5)  # 10 11
    test_graph.add_bidir_lane(5, 6)  # 12 13
    test_graph.add_dock_lane(6, 7, "A")  # 14 15
    test_graph.add_bidir_lane(5, 8)  # 16 17
    test_graph.add_bidir_lane(6, 9)  # 18 19
    test_graph.add_bidir_lane(8, 9)  # 20 21
    test_graph.add_dock_lane(8, 10, "B")  # 22 23

    assert test_graph.num_lanes == 24

    test_graph.add_key(pickup_name, 7)
    test_graph.add_key(dropoff_name, 10)

    assert len(test_graph.keys) == 2 and pickup_name in test_graph.keys \
        and dropoff_name in test_graph.keys

    # INIT FLEET ==============================================================

    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(0.7, 0.3),
                                        angular=traits.Limits(1.0, 0.45),
                                        profile=profile)

    # Manages delivery or loop requests
    adapter = adpt.MockAdapter("test_Delivery")
    fleet = adapter.add_fleet("test_fleet", robot_traits, test_graph)
    fleet.accept_delivery_requests(lambda x: True)

    cmd_node = Node("RobotCommandHandle")
    time_now = datetime.datetime.fromtimestamp(cmd_node
                                               .get_clock()
                                               .now()
                                               .seconds_nanoseconds()[0])

    starts = [plan.Start(datetime.datetime.now() - time_now,
                         0,
                         0.0)]

    # Lambda to insert an adapter
    def updater_inserter(handle_obj, updater):
        handle_obj.updater = updater

    # Manages and executes robot commands
    robot_cmd = MockRobotCommand(cmd_node, test_graph)

    fleet.add_robot(robot_cmd,
                    "T0",
                    profile,
                    starts,
                    partial(updater_inserter, robot_cmd))

    # INIT TASK OBSERVER ======================================================
    at_least_one_incomplete = False
    completed = False

    def task_cb(msg):
        global at_least_one_incomplete
        global completed

        if msg.state == TASK_STATE_COMPLETED:
            completed = True
        else:
            at_least_one_incomplete = True

    task_node = rclpy.create_node("task_summary_node")
    task_node.create_subscription(TaskSummary,
                                  "task_summaries",
                                  task_cb,
                                  10)

    # INIT DISPENSERS =========================================================
    quiet_dispenser = MockQuietDispenser(quiet_dispenser_name)
    flaky_dispenser = MockFlakyDispenser(flaky_dispenser_name)

    # FINAL PREP ==============================================================
    request = adpt.type.DeliveryMsg("test_delivery",
                                    pickup_name,
                                    quiet_dispenser_name,
                                    dropoff_name,
                                    flaky_dispenser_name)

    rclpy_executor = SingleThreadedExecutor()
    # rclpy_executor.add_node(task_node)
    rclpy_executor.add_node(cmd_node)
    rclpy_executor.add_node(quiet_dispenser)
    rclpy_executor.add_node(flaky_dispenser)

    adapter.start()
    adapter.request_delivery(request)
    rclpy_executor.spin_once(100)

    last_quiet_state = None
    last_flaky_state = None

    for i in range(10000):
        if quiet_dispenser.success_flag != last_quiet_state:
            last_quiet_state = quiet_dispenser.success_flag
            print("== QUIET DISPENSER FLIPPED SUCCESS STATE ==",
                  last_quiet_state)

        if flaky_dispenser.success_flag != last_flaky_state:
            last_flaky_state = flaky_dispenser.success_flag
            print("== FLAKY DISPENSER FLIPPED SUCCESS STATE ==",
                  last_flaky_state)

        rclpy_executor.spin_once(100)

        if quiet_dispenser.success_flag and flaky_dispenser.success_flag:
            break


    # rclpy.spin_once(adapter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # quiet_dispenser.destroy_node()
    # flaky_dispenser.destroy_node()
    # quiet_dispenser.destroy_node()

    rclpy.shutdown()
