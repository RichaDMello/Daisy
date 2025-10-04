import traceback
import time
import sys
import subprocess

from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from wasd_wrapper import WasdInterface
import globals
import logging
import signal
import time
import os

# import wasd_kick


from RobotState import RobotState, RobotMode

import bosdyn.api.basic_command_pb2 as basic_command_pb2
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client import frame_helpers, math_helpers 
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.geometry import EulerZXY

class Commander():
    #Idea here: Commander is base class, just prints stuff for testing. Subclass actually commands robot
    def __init__(self, cholder, state):
        self.cholder = cholder
        self.motors_powered = False
        self.state = state
        self.robot = cholder.robot
        self.custom_code = None
        
    def send_state_command(self):
        print(".", end="")
        sys.stdout.flush()



    def initialize(self):
        print("Initialize")
        pass

    def power_motors(self):
        print("Power Motors")
        pass

    def shutdown(self):
        print("Shutdown")
        pass

    def get_status(self):
        print("Get Status")
        pass

    def get_hardware(self):
        print("Get hardware")
        pass

    def get_metrics(self):
        print("Get metrics")
        pass
    
    def estop(self):
        print("ESTOP")
        pass

    def release_estop(self):
        print("Release estop")
        pass
    
    def register_estop(self):
        print("Register estop")
        pass

    def get_robot(self):
        print("Get robot")
        pass

    
    
class RobotCommander(Commander):
    def __init__(self, cholder, state):
        self.cholder = cholder
        self.motors_powered = False
        self.state = state
        self.robot = cholder.robot

    def send_state_command(self):
        print("SENDING STATE COMMAND!")
        try: #TODO move try outside
            state = self.state
            left_x = state.left_x
            left_y = state.left_y
            right_x = state.right_x
            right_y = state.right_y

            #handle resets for Stand mode
            
            if state.mode == RobotMode.Stand:
                #if state.stand_height_change and left_y == 0.0:
                #    self._reset_height()
                if state.stand_roll_change and left_x == 0.0:
                    self._reset_roll()
                if state.stand_pitch_change and right_y == 0.0:
                    self._reset_pitch()
                if state.stand_yaw_change and right_x == 0.0:
                    self._reset_yaw()

            if state.estop_pressed:
                self._toggle_estop()
                #state.estop_pressed = False #?
                
            # Handle 
            #TODO: Change height
            #something like "if the state says the height should change, call self._change_height

            #TODO: battery pose and selfright

            match state.mode:
                case RobotMode.Walk:
                    self._walk()
                    pass
                case RobotMode.Sit:
                    self._sit()
                    pass
                case RobotMode.Stand:
                    self._stand()
                    pass
                case RobotMode.Stairs:
                    pass
                case RobotMode.Jog:
                    self._jog()
                    pass
                case RobotMode.Amble:
                    self._amble()
                    pass
                case RobotMode.Crawl:
                    self._crawl()
                    pass
                case RobotMode.Hop:
                    self._hop()
                    pass

            if state.mode == RobotMode.Stand:
                if left_x != 0.0 or left_y != 0.0 or right_x != 0.0 or right_y != 0.0:
                    self._update_orientation(left_x, left_y, right_x, right_y)
            elif state.mode == RobotMode.Sit:
                pass
            else: #We're moving
                if left_x != 0.0 or left_y != 0.0 or right_x != 0.0:
                    self._move()
                else: #TODO why is this if here?
                    if state.mode == RobotMode.Walk or\
                       state.mode == RobotMode.Amble or\
                       state.mode == RobotMode.Crawl or\
                       state.mode == RobotMode.Jog or\
                       state.mode == RobotMode.Hop:
                        self._move()

            #Handle starting, powering motors...
            #if joy.start():
            #    self._gain_control()
            #    self._power_motors()

        except Exception as e:
            print("Exception encountered in sending state command")
            print(e)
            print(traceback.format_exc())
            

    def initialize(self):
        print("R Initialize")
        pass

    def power_motors(self):
        self.cholder.robot.power_on(timeout_sec=20)
        self.cholder.robot.time_sync.wait_for_sync()
        self.motors_powered = True


    def powerdown_motors(self):
        self.cholder.robot.power_off(cut_immediately=False, timeout_sec=20)
        self.cholder.estop_keep_alive.allow()
        self.motors_powered = False

    def shutdown(self):
        print("R Shutdown")
        self.powerdown_motors()
        self.cholder.lease_keep_alive.shutdown()
        print("is alive:", self.cholder.lease_keep_alive.is_alive())
        
        self.cholder.has_lease_client = False
        

    def get_state(self):
        print(self.state)
        
    def get_status(self):
        print(self.cholder.state_client.get_robot_state())

    def get_hardware(self):
        print("You don't want this...")
        #print(self.cholder.state_client.get_hardware_config_with_link_info())

    def get_metrics(self):
        print(self.cholder.state_client.get_robot_metrics())
    
    def estop(self):
        print("R ESTOP")
        pass

    def release_estop(self):
        print("R Release estop")
        self.cholder.estop_keep_alive.allow()
        pass
    
    def hijack(self):
        print("Hijacking lease")
        # stop_thread.set()
        # allow_sigquit = False
        # # Wait for the worker thread to actually stop
        # if worker_thread:
        #     worker_thread.join()
        self.cholder.lease_client.take()

    def self_right(self):
        print("Activating self right")
        cmd = RobotCommandBuilder.selfright_command()
        self._issue_robot_command(cmd)

    def get_robot(self):
        return self.robot

    def return_to_neutral(self):     
        xo = 0.3
        yo = 0.2
        fl = (xo, yo)
        fr = (xo, -yo)
        hl = (-xo, yo)
        hr = (-xo, -yo)
        self._pose([fl, fr, hl, hr])

    def play_bow(self):     
        fl = (.5, .1)
        fr = (.5, -.1)
        xo = 0.5
        yo = 0.4
        hl = (-xo, yo)
        hr = (-xo, -yo)
        self._pose([fl, fr, hl, hr])

    def intimidate(self):
        xo = 0.5
        yo = 0.4
        fl = (xo, yo)
        fr = (xo, -yo)
        hl = (-xo, yo)
        hr = (-xo, -yo)
        self._pose([fl, fr, hl, hr])

    def run_script(self, script):
        if(self.motors_powered) or (self.cholder.has_lease_client):
            print("Shutting down for new program")
            self.shutdown()
            time.sleep(3)
            # return 
        # if (self.cholder.has_lease_client):
        #     print("Releasing lease")
        #     self.release_estop()
        #     return 
        print("Script on queue: ", script)
        self.custom_code = subprocess.Popen(["python", script, "192.168.80.3"])
        # print("script: ", script)
    
    def on_stop_clicked(self):
        if self.custom_code and self.custom_code.poll() is None:
            self.custom_code.kill()   # or terminate()
    # def run_wasd_wrapper(self):
    #     return True

    def _pose(self, leg_pos):
        xs, ys = zip(*leg_pos)
        if not (all(0.2 <= abs(x) <= 0.5 for x in xs) and all((0.1 <= abs(y) <= 0.4 for y in ys))):
            print("Pose deemed too dangerous")
            return
                    

        vo_T_body = frame_helpers.get_se2_a_tform_b(self.cholder.state_client.get_robot_state().kinematic_state.transforms_snapshot,
                                                    frame_helpers.VISION_FRAME_NAME,
                                                    frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)
        fl, fr, hl, hr = leg_pos
        pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(*fl,0)
        pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(*fr, 0)
        pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(*hl, 0)
        pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(*hr, 0)

        stance_cmd = RobotCommandBuilder.stance_command(
            frame_helpers.VISION_FRAME_NAME, pos_fl_rt_vision.position, pos_fr_rt_vision.position,
            pos_hl_rt_vision.position, pos_hr_rt_vision.position)
        stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
            self.cholder.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))
        
        self._issue_robot_command(stance_cmd)

    def _stand(self):
        ch = self.cholder
        if not self.state.standing:
            self.state.standing = True
            ch.mobility_params = spot_command_pb2.MobilityParams(locomotion_hint=spot_command_pb2.HINT_AUTO)
            #TODO: Figure out why she jerks left when standing if in non-neutral position when she sat
            cmd = RobotCommandBuilder.synchro_stand_command(body_height=self.state.body_height, params=ch.mobility_params)
            self._issue_robot_command(cmd)
            #ch.command_client.robot_command_async(cmd)
        

    def _sit(self):
        self.state.standing = False
        ch = self.cholder
        ch.mobility_params = spot_command_pb2.MobilityParams(locomotion_hint=spot_command_pb2.HINT_AUTO)
        cmd = RobotCommandBuilder.synchro_sit_command(params=ch.mobility_params)
        self._issue_robot_command(cmd)
        #ch.command_client.robot_command_async(cmd)


    def _crawl(self):
        state = self.state
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_CRAWL, stair_hint=0)

        cmd = RobotCommandBuilder.synchro_stand_command(params=self.mobility_params)
        self._issue_robot_command(cmd)
            
        pass

    def _walk(self):
        state = self.state
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT, stair_hint=0)

        cmd = RobotCommandBuilder.synchro_stand_command(params=self.mobility_params)
        self._issue_robot_command(cmd)
            
        pass

    def _amble(self):
        state = self.state
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_AMBLE, stair_hint=0)

        cmd = RobotCommandBuilder.synchro_stand_command(params=self.mobility_params)
        self._issue_robot_command(cmd)
            
        pass

    def _jog(self):
        state = self.state
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_JOG, stair_hint=0)
        cmd = RobotCommandBuilder.synchro_stand_command(params=self.mobility_params)
        self._issue_robot_command(cmd)
        pass

    def _hop(self):
        state = self.state
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_HOP, stair_hint=0)

        cmd = RobotCommandBuilder.synchro_stand_command(params=self.mobility_params)
        self._issue_robot_command(cmd)
            
        pass


    def _move(self):
        print("MOVE")
        state = self.state
        left_x = state.left_x
        left_y = state.left_y
        right_x = state.right_x
        v_y = 0.0
        v_x = 0.0
        v_rot = 0.0
        ch = self.cholder
        
        v_y = -left_x * state.VELOCITY_BASE_SPEED
        v_x = -left_y * state.VELOCITY_BASE_SPEED

        v_rot = -right_x * state.VELOCITY_BASE_ANGULAR

        # Recreate mobility_params with the latest information
        ch.mobility_params = RobotCommandBuilder.mobility_params(
            body_height=state.body_height, locomotion_hint=ch.mobility_params.locomotion_hint,
            stair_hint=ch.mobility_params.stair_hint)

        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot,
                                                           params=self.mobility_params)
        self._issue_robot_command(cmd, endtime=time.time() + state.VELOCITY_CMD_DURATION)


    def _change_height(self, direction):
        """Changes robot body height.
            
        Args:
            direction: 1 to increase height, -1 to decrease height.
        """
        state = self.state
        state.body_height = state.body_height + direction * state.HEIGHT_CHANGE
        state.body_height = min(state.HEIGHT_MAX, state.body_height)
        state.body_height = max(-state.HEIGHT_MAX, state.body_height)

    def _update_orientation(self, left_x, left_y, right_x, right_y):
        """Updates body orientation in Stand mode.

        Args:
            left_x: X value of left stick.
            left_y: Y value of left stick.
            right_x: X value of right stick.
            right_y: Y value of right stick.
        """
        print("Updating orientation: ", left_x, left_y, right_x, right_y)
        state = self.state
        state.stand_roll = left_x/2.1 #/2.1 because of limits on rolling
        #state.body_height = left_y
        
        if left_y != 0:
            print(-left_y)
            self._change_height(-left_y)
        
        state.stand_yaw = right_x
        state.stand_pitch = right_y
        
        self._orientation_cmd_helper(yaw=state.stand_yaw, roll=state.stand_roll,
                                     pitch=state.stand_pitch, height=state.body_height)

    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0.0, height=0.0):
        """Helper function that commands the robot with an orientation command;
        Used by the other orientation functions.

        Args:
            yaw: Yaw of the robot body. Defaults to 0.0.
            roll: Roll of the robot body. Defaults to 0.0.
            pitch: Pitch of the robot body. Defaults to 0.0.
            height: Height of the robot body from normal stand height. Defaults to 0.0.
        """
        state = self.state
        print("Attempting: ", yaw, roll, pitch, height)
        if not self.motors_powered:
            return

        orientation = EulerZXY(-yaw, roll, pitch) #Note: negative pitch to make more sense
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height,
                                                        footprint_R_body=orientation)
        self._issue_robot_command(cmd, endtime=time.time() + state.VELOCITY_CMD_DURATION)

    def _issue_robot_command(self, command, endtime=None):
        """Check that the lease has been acquired and motors are powered on before issuing a command.

        Args:
            command: RobotCommand message to be sent to the robot.
            endtime: Time (in the local clock) that the robot command should stop.
        """

        if not self.cholder.has_command_client:
            print('Must have control by acquiring a lease before commanding the robot.')
            return
        if not self.motors_powered:
            print('Must have motors powered on before commanding the robot.')
            return

        self.cholder.command_client.robot_command_async(command, end_time_secs=endtime)

 
    def battery_change_pose(self, left):
        """Executes the battery-change pose command which causes the robot to sit down if
        standing then roll to its [right]/left side for easier battery changing.
        """
        if left:
            cmd = RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_LEFT)
        else:
            cmd = RobotCommandBuilder.battery_change_pose_command(
            dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT)
        self._issue_robot_command(cmd)


