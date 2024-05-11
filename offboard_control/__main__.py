#! /usr/bin/env python

import argparse
import contextlib
import sys

import cv2.aruco
import cv_bridge
import geometry_msgs.msg
import mavros.mavlink
import mavros_msgs.msg
import mavros_msgs.srv
import numpy
import pymavlink.mavutil
import rospy
import sensor_msgs.msg


def _arguments_aruco_marker_parse(serialized):
    dictionary_name, id_, length = serialized.split(':')
    return _ArucoMarker(dictionary_name=dictionary_name, id_=int(id_), length=float(length))


def _arguments_camera_matrix_parse(serialized):
    flat = [float(element) for element in serialized.split(',')]
    return numpy.array([flat[:3], flat[3:6], flat[6:]], dtype=numpy.float64)


def _arguments_distortion_coefficients_parse(serialized):
    return numpy.array([[float(coefficient) for coefficient in serialized.split(',')]], dtype=numpy.float64)


@contextlib.contextmanager
def _ros_subscribe(topic, message_class, buffer_size=None):
    messages = _Buffer(max_size=buffer_size)
    subscriber = rospy.Subscriber(topic, message_class, callback=messages.append)
    try:
        yield messages
    finally:
        subscriber.unregister()


def _detect_aruco_marker(image, marker, camera_matrix, distortion_coefficients):
    markers_corners, markers_ids, _ = cv2.aruco.detectMarkers(image=cv_bridge.CvBridge().imgmsg_to_cv2(image), dictionary=marker.dictionary)
    assert (markers_corners is None or not len(markers_corners)) == (markers_ids is None or not len(markers_ids))
    if markers_corners is None or not len(markers_corners):
        return None

    marker_indices, _ = numpy.where(markers_ids == marker.id)
    if not marker_indices.size:
        return None

    assert len(markers_corners) == len(markers_ids)
    retval, _, ((marker_x,), (marker_y,), _) = cv2.solvePnP(
        objectPoints=numpy.float32([[-marker.length / 2, marker.length / 2, 0], [marker.length / 2, marker.length / 2, 0], [marker.length / 2, -marker.length / 2, 0], [marker.length / 2, -marker.length / 2, 0]]),
        imagePoints=markers_corners[marker_indices[0]],
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coefficients)
    assert retval
    return marker_x, marker_y


class OffboardControl:
    def __init__(self, camera_matrix, distortion_coefficients, bottom_camera_node_name, mavros_node_name='mavros', mavlink_node_name='mavlink', mav_state_set_rate_hz=1, land_on_aruco_rate_hz=20):
        self._camera_matrix = camera_matrix
        self._distortion_coefficients = distortion_coefficients
        self._mav_state_set_rate = rospy.Rate(mav_state_set_rate_hz)
        self._land_on_aruco_rate = rospy.Rate(land_on_aruco_rate_hz)
        self._mavros_node_name = mavros_node_name
        self._mavros_cmd_arming = rospy.ServiceProxy(f'{self._mavros_node_name}/cmd/arming', mavros_msgs.srv.CommandBool)
        self._mavros_param_set = rospy.ServiceProxy(f'{self._mavros_node_name}/param/set', mavros_msgs.srv.ParamSet)
        self._mavros_set_mode = rospy.ServiceProxy(f'{self._mavros_node_name}/set_mode', mavros_msgs.srv.SetMode)
        self._mavlink_node_name = mavlink_node_name
        self._bottom_camera_node_name = bottom_camera_node_name

    def close(self):
        try:
            self._mavros_cmd_arming.close()
        finally:
            try:
                self._mavros_param_set.close()
            finally:
                self._mavros_set_mode.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def take_off(self):
        self._mav_state_set(landed_state=pymavlink.mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, mode='AUTO.LOITER', armed=True)

    def land_on_aruco(self, marker, search_altitude=None):
        landing_target = None
        landing_target_previous = None

        mavlink_to_publisher = self._mavlink_to_publisher(queue_size=1)
        with self._mavros_state_subscribe(buffer_size=1) as mavros_states, self._mavros_extended_state_subscribe(buffer_size=1) as mavros_extended_states, self._bottom_camera_image_raw_subscribe(buffer_size=1) as bottom_camera_image_raws, self._mavros_local_position_pose_subscribe(buffer_size=1) as mavros_local_position_poses:
            while not rospy.is_shutdown():
                mavros_extended_state_last = mavros_extended_states.get(-1)
                mavros_state_last = mavros_states.get(-1)
                bottom_camera_image_raw_last = bottom_camera_image_raws.get(-1)
                mavros_local_position_pose_last = mavros_local_position_poses.get(-1)

                if mavros_extended_state_last is not None and mavros_extended_state_last.landed_state == pymavlink.mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND and mavros_state_last is not None and mavros_state_last.mode == 'AUTO.LOITER' and not mavros_state_last.armed and mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_STANDBY:
                    break

                if mavros_state_last is None:
                    rospy.loginfo_throttle_identical(1, 'Waiting for MAV state...')
                elif not mavros_state_last.connected:
                    rospy.loginfo_throttle_identical(1, 'Waiting for MAV to connect...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_UNINIT:
                    rospy.loginfo_throttle_identical(1, 'MAV connected. Waiting for initialization...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_BOOT:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and initialized. Waiting for booting up...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_CALIBRATING:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and booted up. Waiting for calibration...')
                elif mavros_state_last.system_status in (pymavlink.mavutil.mavlink.MAV_STATE_EMERGENCY, pymavlink.mavutil.mavlink.MAV_STATE_POWEROFF, pymavlink.mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION):
                    raise NotImplementedError(f'MAV with {mavros_state_last.system_status} system status not supported yet')
                elif mavros_extended_state_last is None:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and calibrated. Waiting for extended state...')
                elif mavros_local_position_pose_last is None:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and calibrated. Waiting for local position...')
                else:
                    try:
                        self._mavros_param_set.wait_for_service()
                    except rospy.ROSInterruptException:
                        raise ValueError('Shut down')

                    if search_altitude is not None:
                        try:
                            mavros_param_set_response = self._mavros_param_set('PLD_SRCH_ALT', mavros_msgs.msg.ParamValue(real=search_altitude))
                        except rospy.ServiceException:
                            raise NotImplementedError(f'Communication with {self._mavros_param_set.resolved_name} service failed')
                        except rospy.ROSInterruptException:
                            raise ValueError('Shut down')
                        if mavros_param_set_response.success and mavros_param_set_response.value.integer == 0 and mavros_param_set_response.value.real == search_altitude:
                            search_altitude = None
                        else:
                            rospy.loginfo('Setting MAV search altitude failed')

                    if mavros_extended_state_last.landed_state != pymavlink.mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                        if mavros_state_last.mode == 'AUTO.PRECLAND' and mavros_state_last.armed:
                            if bottom_camera_image_raw_last is None:
                                rospy.loginfo('MAV connected and calibrated. Waiting for bottom camera image...')
                            else:
                                marker_position_rel = _detect_aruco_marker(image=bottom_camera_image_raw_last, marker=marker, camera_matrix=self._camera_matrix, distortion_coefficients=self._distortion_coefficients)
                                if marker_position_rel is None:
                                    if landing_target_previous is not None:
                                        rospy.loginfo('AruCo marker %s lost.', marker)
                                    landing_target_previous = None
                                else:
                                    landing_target = pymavlink.mavutil.mavlink.MAVLink_landing_target_message(
                                        time_usec=rospy.Time.now().to_nsec() // 1000,
                                        x=mavros_local_position_pose_last.pose.position.y + marker_position_rel[1],
                                        y=mavros_local_position_pose_last.pose.position.x - marker_position_rel[0],
                                        z=0,
                                        position_valid=1,
                                        target_num=0, frame=pymavlink.mavutil.mavlink.MAV_FRAME_LOCAL_NED, angle_x=0, angle_y=0, distance=0, size_x=0, size_y=0)
                                    landing_target.pack(pymavlink.mavutil.mavlink.MAVLink('', 255, 1))
                                    landing_target_ros = mavros.mavlink.convert_to_rosmsg(landing_target)
                                    mavlink_to_publisher.publish(landing_target_ros)

                                    if landing_target_previous is None:
                                        rospy.loginfo('AruCo marker %s found.', marker)
                                    landing_target_previous = landing_target
                        else:
                            rospy.loginfo('MAV connected and calibrated. Landing precisely...')
                            self._mav_state_set(landed_state=mavros_extended_state_last.landed_state, mode='AUTO.PRECLAND', armed=True)
                    else:
                        rospy.loginfo('MAV connected, calibrated and on ground. Switching to standby system status...')
                        self._mav_state_set(landed_state=mavros_extended_state_last.landed_state, mode='AUTO.LOITER', armed=False)

                try:
                    self._land_on_aruco_rate.sleep()
                except rospy.ROSInterruptException:
                    raise ValueError('Shut down')
            else:
                raise ValueError('Shut down')

        rospy.loginfo('MAV landed on x=%f y=%f. Last known position of AruCo marker %s was %s.', mavros_local_position_pose_last.pose.position.x, mavros_local_position_pose_last.pose.position.y, marker, '???' if landing_target is None else 'x=%f y=%f' % (landing_target.y, landing_target.x))

    def _mav_state_set(self, landed_state, mode, armed):
        system_status = pymavlink.mavutil.mavlink.MAV_STATE_ACTIVE if armed else pymavlink.mavutil.mavlink.MAV_STATE_STANDBY

        with self._mavros_state_subscribe(buffer_size=1) as mavros_states, self._mavros_extended_state_subscribe(buffer_size=1) as mavros_extended_states:
            while not rospy.is_shutdown():
                mavros_extended_state_last = mavros_extended_states.get(-1)
                mavros_state_last = mavros_states.get(-1)

                if mavros_extended_state_last is not None and mavros_extended_state_last.landed_state == landed_state and mavros_state_last is not None and mavros_state_last.mode == mode and mavros_state_last.armed == armed and mavros_state_last.system_status == system_status:
                    break

                if mavros_state_last is None:
                    rospy.loginfo_throttle_identical(1, 'Waiting for MAV state...')
                elif not mavros_state_last.connected:
                    rospy.loginfo_throttle_identical(1, 'Waiting for MAV to connect...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_UNINIT:
                    rospy.loginfo_throttle_identical(1, 'MAV connected. Waiting for initialization...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_BOOT:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and initialized. Waiting for booting up...')
                elif mavros_state_last.system_status == pymavlink.mavutil.mavlink.MAV_STATE_CALIBRATING:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and booted up. Waiting for calibration...')
                elif mavros_state_last.system_status in (pymavlink.mavutil.mavlink.MAV_STATE_EMERGENCY, pymavlink.mavutil.mavlink.MAV_STATE_POWEROFF, pymavlink.mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION):
                    raise NotImplementedError(f'MAV with {mavros_state_last.system_status} system status not supported yet')
                elif mavros_extended_state_last is None:
                    rospy.loginfo_throttle_identical(1, 'MAV connected and calibrated. Waiting for extended state...')
                else:
                    try:
                        self._mavros_set_mode.wait_for_service()
                        self._mavros_cmd_arming.wait_for_service()
                    except rospy.ROSInterruptException:
                        raise ValueError('Shut down')

                    if mavros_extended_state_last.landed_state != landed_state:
                        if landed_state == pymavlink.mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                            if mavros_state_last.mode == 'AUTO.TAKEOFF' and mavros_state_last.armed:
                                rospy.loginfo_throttle_identical(1, 'MAV connected and calibrated. Waiting for takeoff...')
                            else:
                                rospy.loginfo('MAV connected and calibrated. Taking off...')
                                self._mav_state_set(landed_state=mavros_extended_state_last.landed_state, mode='AUTO.TAKEOFF', armed=True)
                        else:
                            raise NotImplementedError(f'MAV landed state not supported yet: {landed_state}')
                    elif mavros_state_last.mode != mode:
                        rospy.loginfo('MAV connected, calibrated and in %d landed state. Switching to %s mode...', landed_state, mode)
                        try:
                            mavros_set_mode_response = self._mavros_set_mode(custom_mode=mode)
                        except rospy.ServiceException:
                            raise NotImplementedError(f'Communication with {self._mavros_set_mode.resolved_name} service failed')
                        except rospy.ROSInterruptException:
                            raise ValueError('Shut down')
                        if not mavros_set_mode_response.mode_sent:
                            rospy.loginfo('Switching MAV to %s mode failed', mode)
                    elif mavros_state_last.armed != armed:
                        rospy.loginfo('MAV connected, calibrated, in %d landed state and in %s mode. %s...', landed_state, mode, 'Arming' if armed else 'Disarming')
                        try:
                            mavros_cmd_arming_response = self._mavros_cmd_arming(value=armed)
                        except rospy.ServiceException:
                            raise NotImplementedError(f'Communication with {self._mavros_cmd_arming.resolved_name} service failed')
                        except rospy.ROSInterruptException:
                            raise ValueError('Shut down')
                        if not mavros_cmd_arming_response.success or not mavros_cmd_arming_response.result:
                            rospy.logwarn('%s MAV failed: %d', 'Arming' if armed else 'Disarming', mavros_cmd_arming_response.result)
                    elif mavros_state_last.system_status != system_status:
                        rospy.loginfo_throttle_identical(1, 'MAV connected, calibrated, in %d landed state, in %s mode and %s. Waiting for %d system status...', landed_state, mode, 'armed' if armed else 'disarmed', system_status)

                try:
                    self._mav_state_set_rate.sleep()
                except rospy.ROSInterruptException:
                    raise ValueError('Shut down')
            else:
                raise ValueError('Shut down')

        rospy.loginfo('MAV %d landed state, %s mode, %s and %d system status set.', mavros_extended_state_last.landed_state, mavros_state_last.mode, 'arm' if armed else 'disarm', system_status)

    def _bottom_camera_image_raw_subscribe(self, buffer_size=None):
        return _ros_subscribe(topic=f'{self._bottom_camera_node_name}/image_raw', message_class=sensor_msgs.msg.Image, buffer_size=buffer_size)

    def _mavlink_to_publisher(self, queue_size):
        return rospy.Publisher(f'{self._mavlink_node_name}/to', mavros_msgs.msg.Mavlink, queue_size=queue_size)

    def _mavros_extended_state_subscribe(self, buffer_size=None):
        return _ros_subscribe(topic=f'{self._mavros_node_name}/extended_state', message_class=mavros_msgs.msg.ExtendedState, buffer_size=buffer_size)

    def _mavros_local_position_pose_subscribe(self, buffer_size=None):
        return _ros_subscribe(topic=f'{self._mavros_node_name}/local_position/pose', message_class=geometry_msgs.msg.PoseStamped, buffer_size=buffer_size)

    def _mavros_state_subscribe(self, buffer_size=None):
        return _ros_subscribe(topic=f'{self._mavros_node_name}/state', message_class=mavros_msgs.msg.State, buffer_size=buffer_size)


class _ArucoMarker:
    def __init__(self, dictionary_name, id_, length):
        self._dictionary_name = dictionary_name
        self._dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f'DICT_{self.dictionary_name}'))
        self._id = id_
        self._length = length

    def __str__(self):
        return f'{self.dictionary_name}:{self.id}'

    @property
    def dictionary_name(self):
        return self._dictionary_name

    @property
    def dictionary(self):
        return self._dictionary

    @property
    def id(self):
        return self._id

    @property
    def length(self):
        return self._length


class _Buffer:
    def __init__(self, max_size=None):
        self._max_size = max_size
        self._items = []

    def __len__(self):
        return len(self._items)

    def get(self, index, default=None):
        try:
            return self._items[index]
        except IndexError:
            return default

    def append(self, item):
        while self._max_size is not None and len(self) >= self._max_size:
            self._items.pop(0)
        self._items.append(item)


if __name__ == "__main__":
    rospy.init_node("offboard_control")

    arguments_parser = argparse.ArgumentParser()
    arguments_parser_command = arguments_parser.add_subparsers()
    arguments_parser_command_precland = arguments_parser_command.add_parser('precland')
    arguments_parser_command_precland.add_argument('--distortion-coefficients', default='0,0,0,0', type=_arguments_distortion_coefficients_parse)
    arguments_parser_command_precland.add_argument('--search-altitude', type=float)
    arguments_parser_command_precland.add_argument('--mavros-node-name', default='mavros')
    arguments_parser_command_precland.add_argument('--mavlink-node-name', default='mavlink')
    arguments_parser_command_precland.add_argument('--mav-state-set-rate', default='1', type=float)
    arguments_parser_command_precland.add_argument('--land-on-aruco-rate', default='20', type=float)
    arguments_parser_command_precland.add_argument('camera_matrix', type=_arguments_camera_matrix_parse)
    arguments_parser_command_precland.add_argument('bottom_camera_node_name')
    arguments_parser_command_precland.add_argument('aruco_markers', nargs='*', type=_arguments_aruco_marker_parse)
    arguments = arguments_parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    with OffboardControl(camera_matrix=arguments.camera_matrix, distortion_coefficients=arguments.distortion_coefficients, mavros_node_name=arguments.mavros_node_name, mavlink_node_name=arguments.mavlink_node_name, bottom_camera_node_name=arguments.bottom_camera_node_name, mav_state_set_rate_hz=arguments.mav_state_set_rate, land_on_aruco_rate_hz=arguments.land_on_aruco_rate) as offboard_control:
        for aruco_marker in arguments.aruco_markers:
            rospy.loginfo('Landing on AruCo marker %s...', aruco_marker)
            offboard_control.take_off()
            offboard_control.land_on_aruco(search_altitude=arguments.search_altitude, marker=aruco_marker)
            rospy.loginfo('Landing on AruCo marker %s finished.', aruco_marker)

    rospy.spin()
