import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy
import rosparam
import roslaunch.substitution_args

from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, ListControllers

LOADED_ARM_CONTROLLERS = ['%s_arm_controller', '%s_cart']
LOADED_CTRLS_PARAMS = {
    'r' : "/controller_manager/loaded_ctrls/right_arm",
    'l' : "/controller_manager/loaded_ctrls/left_arm",
}

##
# Offers controller switching inside python on the fly.
class ControllerSwitcher:
    def __init__(self):
        self.load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', 
                                                  LoadController)
        self.unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', 
                                                    UnloadController)
        self.switch_controller_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', 
                                                        SwitchController)
        self.list_controllers_srv = rospy.ServiceProxy('pr2_controller_manager/list_controllers', 
                                                       ListControllers)
        self.load_controller.wait_for_service()
        self.unload_controller.wait_for_service()
        self.switch_controller_srv.wait_for_service()
        self.list_controllers_srv.wait_for_service()
        for arm in ['r', 'l']:
            if LOADED_CTRLS_PARAMS[arm] not in rosparam.list_params(""):
                possible_ctrls = []
                for ctrl in LOADED_ARM_CONTROLLERS:
                    possible_ctrls.append(ctrl % arm)
                rosparam.set_param_raw(LOADED_CTRLS_PARAMS[arm], possible_ctrls)
        rospy.loginfo("[pr2_controller_switcher] ControllerSwitcher ready.")

    ##
    # Switches controller.
    # @param old_controller Name of controller to terminate.
    # @param new_controller Name of controller to activate.  Can be same as old_controller
    #                       if the object is to only change parameters.
    # @param param_file YAML file containing parameters for the new controller.
    # @return Success of switch.
    def switch(self, old_controller, new_controller, param_file=None):
        if param_file is None:
            self.load_controller(new_controller)
            resp = self.switch_controller_srv([new_controller], [old_controller], 1)
            self.unload_controller(old_controller)
            return resp.ok
        else:
            params = rosparam.load_file(roslaunch.substitution_args.resolve_args(param_file))
            rosparam.upload_params("", params[0][0])
            self.switch_controller_srv([], [old_controller], 1)
            self.unload_controller(old_controller)
            if old_controller != new_controller:
                self.unload_controller(new_controller)
            self.load_controller(old_controller)
            if old_controller != new_controller:
                self.load_controller(new_controller)
            resp = self.switch_controller_srv([new_controller], [], 1)
            return resp.ok

    ##
    # Switches controller without having to specify the arm controller to take down.
    # @param arm (r/l)
    # @param new_controller Name of new controller to load
    # @param param_file YAML file containing parameters for the new controller.
    # @param reset If true, the controller will bring down, unload and restart the controller
    #              using new parameters if currently running.  If false, nothing will happen.
    # @return Success of switch.
    def carefree_switch(self, arm, new_controller, param_file=None, reset=True):
        if '%s' in new_controller:
            new_ctrl = new_controller % arm
        else:
            new_ctrl = new_controller
        if param_file is not None:
            params = rosparam.load_file(roslaunch.substitution_args.resolve_args(param_file))
            if new_ctrl not in params[0][0]:
                rospy.logwarn("[pr2_controller_switcher] Controller not in parameter file.")
                return
            else:
                rosparam.upload_params("", {new_ctrl : params[0][0][new_ctrl]})
        possible_controllers = rosparam.get_param(LOADED_CTRLS_PARAMS[arm])
        if new_ctrl not in possible_controllers:
            possible_controllers.append(new_ctrl)
            rosparam.set_param_raw(LOADED_CTRLS_PARAMS[arm], possible_controllers)
        check_arm_controllers = []
        for controller in possible_controllers:
            if '%s' in controller:
                controller = controller % arm 
            if controller[0] == arm:
                check_arm_controllers.append(controller)
        resp = self.list_controllers_srv()
        start_controllers, stop_controllers = [new_ctrl], []
        for i, controller in enumerate(resp.controllers):
            if controller in check_arm_controllers and resp.state[i] == 'running':
                stop_controllers.append(controller)
            if controller == new_ctrl:
                if resp.state[i] == 'running':
                    if not reset:
                        rospy.loginfo("[pr2_controller_switcher] Specified controller is already running.")
                        return False
                    self.switch_controller_srv([], [new_ctrl], 1)
                self.unload_controller(new_ctrl)
                
        self.load_controller(new_ctrl)
        rospy.loginfo("[pr2_controller_switcher] Starting controller %s" % (start_controllers[0]) +
                      " and stopping controllers: [" + ",".join(stop_controllers) + "]")
        resp = self.switch_controller_srv(start_controllers, stop_controllers, 1)
        return resp.ok

    ##
    # Lists the controllers which are currently running.
    # @return List of controller names.
    def list_running(self):
        resp = self.list_controllers_srv()
        running_ctrls = []
        for controller, state in zip(resp.controllers, resp.state):
            if state == 'running':
                running_ctrls.append(controller)
        return running_ctrls

    ##
    # Lists the controllers which are currently stopped.
    # @return List of controller names.
    def list_stopped(self):
        resp = self.list_controllers_srv()
        running_ctrls = []
        for controller, state in zip(resp.controllers, resp.state):
            if state == 'stopped':
                running_ctrls.append(controller)
        return running_ctrls

    ##
    # Lists the controllers which are currently loaded.
    # @return List of controller names.
    def list_loaded(self):
        resp = self.list_controllers_srv()
        return resp.controllers
