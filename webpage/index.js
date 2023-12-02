const express = require('express')
const path = require('path');
const ROSLIB = require('roslib')
const app = express()
const port = 3000

var ros_server = 'ws://localhost:9090';
var ros = new ROSLIB.Ros();
ros.connect(ros_server);

var ledState = new ROSLIB.Topic({
    ros : ros,
    name : '/microROS/led',
    messageType : 'std_msgs/msg/Int16'
  });

var coordState = new ROSLIB.Topic({
    ros : ros,
    name : '/microROS/coord',
    messageType : 'geometry_msgs/msg/Vector3'
  });

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '/index.html'));
})

app.use(express.json());
app.use(express.urlencoded({ extended: true }));

app.post('/LED', (req, res) => {

    var ledAction = new ROSLIB.Message({
        data : parseInt(req.body.data)
      });

    ledState.publish(ledAction);
    //console.log(req.body);
    res.redirect('/');
})

app.post('/COORD', (req, res) => {

  var coordAction = new ROSLIB.Message({
      x : parseFloat(req.body.x),
      y : parseFloat(req.body.y)
    });

    coordState.publish(coordAction);
    console.log(coordAction);
    res.sendStatus(200);
  //console.log(req.body);
})


app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})

