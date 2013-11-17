$(function() {

  var PORT = "8888";
  var HOSTNAME = location.hostname;
  
  var ros = new ROSLIB.Ros({
    url: "ws://" + HOSTNAME + ":" + PORT
  });
  

  var reset_service = new ROSLIB.Service({
    ros: ros,
    name: "/triangle_screenpoint/cancel",
    serviceType: '/triangle_screenpoint/cancel'
  });
  
  var go_service = new ROSLIB.Service({
    ros: ros,
    name: "/triangle_screenpoint/go",
    serviceType: '/triangle_screenpoint/go'
  });
  
  $("#go-button").click(function() {
    var req = new ROSLIB.ServiceRequest({});
    go_service.callService(req, function(){});
  });
  $("#reset-button").click(function() {
    var req = new ROSLIB.ServiceRequest({});
    reset_service.callService(req, function(){});
  });
                           
  
});