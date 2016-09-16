function ($, ROSLIB){
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
        var relaunchVCIService = new ROSLIB.Service({
            ros: ros,
            name:'/relaunch_vci',
            serviceType: 'std_srvs/Empty'}
        );

        var relaunchVCI = function () {
            relaunchVCIService.callService({});
        };

        var robotStartService = new ROSLIB.Service({
            ros: ros,
            name:'/robot_start',
            serviceType: 'std_srvs/Empty'}
        );
        var runRobotStart = function () {
            robotStartService.callService({});
        };

    });
}(jQuery, ROSLIB));
