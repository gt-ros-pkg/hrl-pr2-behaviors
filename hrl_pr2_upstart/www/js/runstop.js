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
        
        $('#stop-button').button().on('mousedown.runstop', stop);
        $('#reset-button').button().on('click.runstop', reset);

    });
}(jQuery, ROSLIB));
