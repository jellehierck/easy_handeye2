import itertools
import math
import threading

import rclpy
import rclpy.node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import std_msgs
import easy_handeye2_msgs.srv
import easy_handeye2_msgs.action
from rclpy.executors import ExternalShutdownException
import std_msgs.msg

import easy_handeye2 as hec
from easy_handeye2.handeye_calibration import save_calibration, HandeyeCalibrationParametersProvider
from easy_handeye2.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
from easy_handeye2.handeye_sampler import HandeyeSampler


class HandeyeServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_server')

        self.parameters_provider = HandeyeCalibrationParametersProvider(self)
        self.parameters = self.parameters_provider.read()

        self.get_logger().info(f'Read parameters for calibration "{self.parameters.name}"')

        self.sampler = HandeyeSampler(self, handeye_parameters=self.parameters)
        self.setup_timer = self.create_timer(2.0, self.setup_services_and_topics)

        self.calibration_backends = {'OpenCV': HandeyeCalibrationBackendOpenCV()}
        self.calibration_algorithm = 'OpenCV/Tsai-Lenz'

        # setup calibration services and topics
        self.list_algorithms_service = None
        self.set_algorithm_service = None
        self.get_current_transforms_service = None
        self.get_sample_list_service = None
        self.take_sample_service = None
        self.remove_sample_service = None
        self.save_samples_service = None
        self.load_samples_service = None
        self.compute_calibration_service = None
        self.save_calibration_service = None
        self.take_sample_topic = None
        self.remove_last_sample_topic = None

        self.last_calibration = None

        # Take multiple samples action
        # Only one goal is executed at the same time, if any are running, new ones are rejected
        # Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        # The start of the action is also optionally deferred for a time
        # Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_defer.py
        self.take_multiple_samples_action_server = None
        self._take_multiple_samples_action_goal_handle = None
        self._take_multiple_samples_action_goal_lock = threading.Lock()
        self._take_multiple_samples_action_defer_timer = None

    def setup_services_and_topics(self):
        if not self.sampler.wait_for_tf_init():
            self.get_logger().warn('Waiting for TF initialization...')
            return

        self.list_algorithms_service = self.create_service(easy_handeye2_msgs.srv.ListAlgorithms, hec.LIST_ALGORITHMS_TOPIC,
                                                           self.list_algorithms)
        self.set_algorithm_service = self.create_service(easy_handeye2_msgs.srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC,
                                                         self.set_algorithm)
        self.get_current_transforms_service = self.create_service(easy_handeye2_msgs.srv.TakeSample, hec.GET_CURRENT_TRANSFORMS_TOPIC,
                                                           self.get_current_transforms)
        self.get_sample_list_service = self.create_service(easy_handeye2_msgs.srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC,
                                                           self.get_sample_lists)
        self.take_sample_service = self.create_service(easy_handeye2_msgs.srv.TakeSample, hec.TAKE_SAMPLE_TOPIC, self.take_sample_srv_callback)
        
        self.remove_sample_service = self.create_service(easy_handeye2_msgs.srv.RemoveSample, hec.REMOVE_SAMPLE_TOPIC,
                                                         self.remove_sample_srv_callback)
        self.save_samples_service = self.create_service(easy_handeye2_msgs.srv.SaveSamples, hec.SAVE_SAMPLES_TOPIC,
                                                        self.save_samples)
        self.load_samples_service = self.create_service(easy_handeye2_msgs.srv.LoadSamples, hec.LOAD_SAMPLES_TOPIC,
                                                        self.load_samples)
        self.compute_calibration_service = self.create_service(easy_handeye2_msgs.srv.ComputeCalibration,
                                                               hec.COMPUTE_CALIBRATION_TOPIC, self.compute_calibration)
        self.save_calibration_service = self.create_service(easy_handeye2_msgs.srv.SaveCalibration, hec.SAVE_CALIBRATION_TOPIC,
                                                            self.save_calibration)

        # Action server for taking multiple samples, but only one goal can be active at the same time
        # Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        self.take_multiple_samples_action_server = ActionServer(
            self,
            easy_handeye2_msgs.action.TakeMultipleSamples,
            hec.TAKE_MULTIPLE_SAMPLES_TOPIC,
            goal_callback=self.take_multiple_samples_goal_callback,
            handle_accepted_callback=self.take_multiple_samples_goal_accepted_callback,
            execute_callback=self.take_multiple_samples_execute_callback,
            cancel_callback=self.take_multiple_samples_cancel_callback,
            callback_group=ReentrantCallbackGroup(),  # Allow callbacks in parallel!
        )

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = self.create_subscription(std_msgs.msg.Empty, hec.TAKE_SAMPLE_TOPIC, self.take_sample_msg_callback,
                                                          10)
        self.remove_last_sample_topic = self.create_subscription(std_msgs.msg.Empty, hec.REMOVE_SAMPLE_TOPIC,
                                                                  self.remove_last_sample, 10)
        self.setup_timer.cancel()

    def destroy(self) -> None:
        """Clean up the node, which also removes the action server."""
        if self.take_multiple_samples_action_server:
            self.take_multiple_samples_action_server.destroy()
        super().destroy_node()

    # algorithm

    def list_algorithms(self, _, response: easy_handeye2_msgs.srv.ListAlgorithms.Response):
        algorithms_nested = [[bck_name + '/' + alg_name for alg_name in bck.AVAILABLE_ALGORITHMS] for bck_name, bck in
                             self.calibration_backends.items()]
        available_algorithms = list(itertools.chain(*algorithms_nested))
        response.algorithms = available_algorithms
        response.current_algorithm = self.calibration_algorithm
        return response

    def set_algorithm(self, req: easy_handeye2_msgs.srv.SetAlgorithm.Request, response: easy_handeye2_msgs.srv.SetAlgorithm.Response):
        alg_to_set = req.new_algorithm
        bckname_algname = alg_to_set.split('/')
        if len(bckname_algname) != 2:
            response.success = False
            return response
        bckname, algname = bckname_algname
        if bckname not in self.calibration_backends:
            response.success = False
            return response
        if algname not in self.calibration_backends[bckname].AVAILABLE_ALGORITHMS:
            response.success = False
            return response
        self.get_logger().info('switching to calibration algorithm {}'.format(alg_to_set))
        self.calibration_algorithm = alg_to_set
        response.success = True
        return response

    # sampling

    def _retrieve_sample_list(self):
        return self.sampler.get_samples()

    def get_current_transforms(self, _, response: easy_handeye2_msgs.srv.TakeSample.Response):
        transforms = self.sampler.current_transforms()
        if transforms is None:
            response.samples.samples = []
            return response

        response.samples.samples = [transforms]
        return response

    def get_sample_lists(self, _, response: easy_handeye2_msgs.srv.TakeSample.Response):
        response.samples = self._retrieve_sample_list()
        return response

    def take_sample_srv_callback(self, _, response: easy_handeye2_msgs.srv.TakeSample.Response):
        self.sampler.take_sample()
        response.samples = self._retrieve_sample_list()
        return response
    
    def take_sample_msg_callback(self, _):
        self.sampler.take_sample()

    def remove_last_sample(self, _):
        self.sampler.remove_sample(len(self.sampler.samples) - 1)

    def remove_sample_srv_callback(self, req: easy_handeye2_msgs.srv.RemoveSample.Request, response: easy_handeye2_msgs.srv.RemoveSample.Response):
        try:
            self.sampler.remove_sample(req.sample_index)
        except IndexError:
            self.get_logger().err('Invalid index ' + req.sample_index)
        response.samples = self._retrieve_sample_list()
        return response

    def save_samples(self, _: easy_handeye2_msgs.srv.SaveSamples.Request, response: easy_handeye2_msgs.srv.SaveSamples.Response):
        try:
            response.success = self.sampler.save_samples()
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response

    def load_samples(self, _: easy_handeye2_msgs.srv.LoadSamples.Request, response: easy_handeye2_msgs.srv.LoadSamples.Response):
        try:
            response.success = self.sampler.load_samples()
            response.samples = self._retrieve_sample_list()
        except IndexError:
            response.success = False
        return response

    # Take multiple samples action callbacks

    def take_multiple_samples_goal_callback(
        self, goal_request: easy_handeye2_msgs.action.TakeMultipleSamples.Goal
    ) -> GoalResponse:
        """Accept a new action goal request or reject it if there is already a running action goal.

        Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        """
        # Check that the input arguments are valid
        if goal_request.max_samples == 0 and goal_request.max_duration == 0.0:
            self.get_logger().error(
                "Bad request: cannot take samples, at least one of 'max_samples' or 'max_duration' must be non-zero."
            )
            return GoalResponse.REJECT

        # This server only allows one goal at a time
        with self._take_multiple_samples_action_goal_lock:
            if (
                self._take_multiple_samples_action_goal_handle is not None
                and self._take_multiple_samples_action_goal_handle.is_active
            ):
                # Reject the new goal
                self.get_logger().warning(
                    f"Cannot start a new {easy_handeye2_msgs.action.TakeMultipleSamples.__name__} action: "
                    "another one is currently running!"
                )
                return GoalResponse.REJECT

        # All checks are passed, the goal can be accepted
        return GoalResponse.ACCEPT

    def take_multiple_samples_goal_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Prepare execution after a goal is accepted.

        The start of the goal execution can be deffered for some time.
        Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_defer.py
        """
        # Set the goal handle to ensure that only one goal is active at the same time
        with self._take_multiple_samples_action_goal_lock:
            # TODO: There could be a race condition here as the lock is yielded between the goal_callback and this one.
            # Possibly need to check again if the goal is already accepted and reject it in that case.
            # Similar to https://github.com/ros2/examples/blob/a7cfa3a9c2b9aafd10b30172edb78091ac656ec3/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py#L57
            self._take_multiple_samples_action_goal_handle = goal_handle

        goal_request: easy_handeye2_msgs.action.TakeMultipleSamples.Goal = goal_handle.request

        # Create a single-fire timer which waits for a little while before starting executing the goal handle
        self.get_logger().info(f"Action will start in {goal_request.defer_action_time} seconds")
        self._take_multiple_samples_action_defer_timer = self.create_timer(
            goal_request.defer_action_time, self.take_multiple_samples_defer_timer_callback
        )

    def take_multiple_samples_defer_timer_callback(self) -> None:
        """Start executing the deferred goal.

        Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_defer.py
        """
        # Cancel the one-shot timer
        if self._take_multiple_samples_action_defer_timer is not None:
            self._take_multiple_samples_action_defer_timer.cancel()

        # Start executing the goal handle
        if self._take_multiple_samples_action_goal_handle is not None:
            self._take_multiple_samples_action_goal_handle.execute()

    def take_multiple_samples_execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> easy_handeye2_msgs.action.TakeMultipleSamples.Result:
        """Execute the action.

        Only one goal can be active at the same time.
        Based on https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        """
        goal_request: easy_handeye2_msgs.action.TakeMultipleSamples.Goal = goal_handle.request

        if goal_request.clear_samples_before:
            self.get_logger().info("Clearing current samples")
            self.sampler.clear_samples()

        # Calculate the number of samples based on the duration or max samples, or inf if no max is given
        max_samples_for_duration = (
            goal_request.sample_frequency * goal_request.max_duration if goal_request.max_duration > 0 else float("inf")
        )
        max_samples = goal_request.max_samples if goal_request.max_samples > 0 else float("inf")

        # Determine which of the two maxes gave the lowest number of samples (cannot both be inf) and round down
        total_nr_samples: int = math.floor(min(max_samples_for_duration, max_samples))
        sample_period: float = 1 / goal_request.sample_frequency
        total_duration: float = total_nr_samples * sample_period

        # Use a Rate object to determine timing for taking samples
        sample_rate = self.create_rate(goal_request.sample_frequency)

        self.get_logger().info(
            f"Starting action: will take sample every {sample_period} sec "
            f"(total {total_nr_samples} samples over {total_duration} sec)"
        )

        for sample_nr in range(1, total_nr_samples + 1):
            # If a cancel request is sent, stop execution
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning("Goal was cancelled!")
                return easy_handeye2_msgs.action.TakeMultipleSamples.Result(samples=self._retrieve_sample_list())

            # If goal is flagged as no longer active, abort execution
            if not goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().warning("Goal was aborted!")
                return easy_handeye2_msgs.action.TakeMultipleSamples.Result(samples=self._retrieve_sample_list())

            # Take next sample
            self.get_logger().info(f"Taking sample {sample_nr}/{total_nr_samples}")
            self.sampler.take_sample()

            feedback_msg = easy_handeye2_msgs.action.TakeMultipleSamples.Feedback(
                partial_samples=self._retrieve_sample_list()
            )
            goal_handle.publish_feedback(feedback_msg)

            # Wait until the next sample can be taken
            sample_rate.sleep()

        self.get_logger().info("All samples are taken!")

        # All samples are taken, construct the result message
        goal_result = easy_handeye2_msgs.action.TakeMultipleSamples.Result(samples=self._retrieve_sample_list())

        # Check if the calibration should be computed
        if goal_request.compute_calibration:
            self.get_logger().info("Computing calibration result...")
            compute_calibration_req = easy_handeye2_msgs.srv.ComputeCalibration.Request()
            compute_calibration_res = easy_handeye2_msgs.srv.ComputeCalibration.Response()
            compute_calibration_res= self.compute_calibration(compute_calibration_req, compute_calibration_res)
            goal_result.calibration_valid = compute_calibration_res.valid
            goal_result.calibration_result = compute_calibration_res.calibration

            # Also check if the calibration should be saved
            if goal_request.save_calibration:
                self.get_logger().info("Saving calibration result to disk...")
                save_calibration_req = easy_handeye2_msgs.srv.SaveCalibration.Request()
                save_calibration_res = easy_handeye2_msgs.srv.SaveCalibration.Response()
                save_calibration_res= self.save_calibration(save_calibration_req, save_calibration_res)
                goal_result.save_calibration_success = save_calibration_res.success
                goal_result.save_calibration_filepath = save_calibration_res.filepath

        # Check if the samples should be saved to disk
        if goal_request.save_samples:
            self.get_logger().info("Saving samples to disk...")
            save_samples_req = easy_handeye2_msgs.srv.SaveSamples.Request()
            save_samples_res = easy_handeye2_msgs.srv.SaveSamples.Response()
            save_samples_res= self.save_samples(save_samples_req, save_samples_res)
            goal_result.save_samples_success = save_samples_res.success

        # Do one final check that the action is still going before signaling that it is complete
        with self._take_multiple_samples_action_goal_lock:
            # Check that the action is not aborted right before the final result is published
            if not goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().warning("Goal was aborted!")
                return easy_handeye2_msgs.action.TakeMultipleSamples.Result(samples=self._retrieve_sample_list())

            # The action is finished gracefully
            goal_handle.succeed()

        # Return result message
        self.get_logger().info("Multiple samples action succeeded!")
        return goal_result

    def take_multiple_samples_cancel_callback(self, _) -> CancelResponse:
        """Any action cancel requests are accepted, the next trigger of the execute function will stop."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # calibration

    def compute_calibration(self, _, response: easy_handeye2_msgs.srv.ComputeCalibration.Response):
        samples = self.sampler.get_samples()

        bckname, algname = self.calibration_algorithm.split('/')
        backend = self.calibration_backends[bckname]

        self.last_calibration = backend.compute_calibration(self, self.parameters, samples, algorithm=algname)
        if self.last_calibration is None:
            self.get_logger().warn('No valid calibration computed')
            response.valid = False
            return response
        response.valid = True
        response.calibration = self.last_calibration
        return response

    def save_calibration(self, _, response: easy_handeye2_msgs.srv.SaveCalibration.Response):
        response.success = False
        if self.last_calibration:
            try:
                filepath = save_calibration(self.last_calibration)
                self.get_logger().info(f'Calibration saved to {filepath}')
                response.success = True
                response.filepath.data = str(filepath)
            except Exception as e:
                self.get_logger().error(f'Could not save calibration')
                self.get_logger().error(f'Underlying exception: {e}')
        return response

    # TODO: evaluation


def main(args=None):
    rclpy.init(args=args)

    handeye_server = HandeyeServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(handeye_server)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_server.destroy_node()


if __name__ == '__main__':
    main()
