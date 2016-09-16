(function ($, ROSLIB){
    var ROBOT = window.location.host.split(':')[0];//Use localhost when serving website directly from robot 
    var PORT = '1820';//Must match port on which rosbridge is being served
    var ros = new ROSLIB.Ros({url: 'ws://'+ ROBOT + ':'+ PORT});
    ros.on('close', function(e) {
       console.log("Disconnected or Can't Connect to " + ROBOT + ":"+ PORT + ".");
       $('#message').html("<h1>Lost Connection to Robot<h1>");
       $('button').addClass('no-connect');
      }
    );

    ros.on('error', function(e) {
      console.log("Rosbridge Connection Error!");
      $('#message').html("<h1>Lost Connection to Robot<h1>");
      $('button').addClass('no-connect');
      }
    );

    ros.on('connection', function(e) {
        console.log("Connected to " + ROBOT + ".");
        $('#message').empty();
        $('button').removeClass('no-connect');

        // Connection to runstop emulator service
        var emulatorService = new ROSLIB.Service({
            ros: ros,
            name:'/emulate_runstop',
            serviceType: 'hrl_pr2_upstart/SetRunStop'}
        );

        var stop = function(event) {
            emulatorService.callService({'stop': true, 'start': false});
        };

        var reset = function(event) {
            emulatorService.callService({'stop': false, 'start': true});
        };

        // Handle motor state (halted vs. running)
        var motorsHalted = null;
        var updateMotorState = function (boolMsg) {
            motorsHalted = boolMsg.data;
            if (motorsHalted) {
                $('#runstop-button').removeClass('stop').addClass('reset');
                $('#runstop-button > span').text('Re-enable Motors');
            } else {
                $('#runstop-button').removeClass('reset').addClass('stop');
                $('#runstop-button > span').text('Halt Motors');
            }
        };

        var motorStateSub = new ROSLIB.Topic({
            ros: ros,
            name: "/pr2_etherCAT/motors_halted",
            messageType: 'std_msgs/Bool'
        });
        motorStateSub.subscribe(updateMotorState);

        var runstopToggleCB = function (event) {
            var fn = motorsHalted ? reset : stop; 
            fn();
        };

        $('#runstop-button').button().on('mousedown.runstop', runstopToggleCB);


        // Handle Calibration State
        var calibrated = null;
        var updateCalibrationStatus = function (boolMsg) {
            if (boolMsg.data) {
                $('#calibration-status').text('Calibration: Complete');
            } else {
                $('#calibration-status').text('Calibration: NOT COMPLETE');
            }
        };

        var calibrationStateSub = new ROSLIB.Topic({
            ros: ros,
            name: '/calibrated',
            messageType: 'std_msgs/Bool'
        });
        calibrationStateSub.subscribe(updateCalibrationStatus);

        // Connection to software reset services
        var stopVCIService = new ROSLIB.Service({
            ros: ros,
            name:'/stop_vci',
            serviceType: 'std_srvs/Trigger'}
        );

        var startVCIService = new ROSLIB.Service({
            ros: ros,
            name:'/start_vci',
            serviceType: 'std_srvs/Empty'}
        );

        var triggerCB = function (resp) {
            console.log("Trigger Succeeded: ", resp.success);
            console.log("Trigger Msg: ", resp.message);
        };

        $('#iface_link > a').attr('href', 'http://' + window.location.hostname+':8008/assistive_teleop/vci.html');
        var vci_running = false;
        var updateVCIRunning = function (boolMsg) {
            vci_running = boolMsg.data;
            if (vci_running) {
                $('#iface_link').show();
                $('#relaunch-vci-button').removeClass('stopped').addClass('running');
                $('#relaunch-vci-button > span').text('Shutdown Web Interface');
            } else {
                $('#iface_link').hide();
                $('#relaunch-vci-button').removeClass('running').addClass('stopped');
                $('#relaunch-vci-button > span').text('Start Web Interface');
            }
        };

        var vciRunningSub = new ROSLIB.Topic({
            ros: ros, 
            name: "/vci_running",
            messageType: "std_msgs/Bool"
        });
        vciRunningSub.subscribe(updateVCIRunning);

        var stopStartVCICB = function (event) {
            if (vci_running) {
                stopVCIService.callService({}, triggerCB);
            } else {
                startVCIService.callService({}, triggerCB);
            }
        };
        $('#relaunch-vci-button').button().on('click', stopStartVCICB);

        var robotStartService = new ROSLIB.Service({
            ros: ros,
            name:'/robot_start',
            serviceType: 'std_srvs/Trigger'}
        );

        var callRobotStart = function (event) {
            var confirmed = confirm("Are you sure you want to restart all the robot's software? This will cause the motors to lose power briefly.");
            if (confirmed) {
                robotStartService.callService({});
            }
        };

        $('#robot-start-button').button().on('click', callRobotStart);
        

    });
}(jQuery, ROSLIB));
