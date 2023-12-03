const express = require('express')
const path = require('path');
const ROSLIB = require('roslib')
const app = express()
const port = 3000

var ros_server = 'ws://localhost:9090';
var ros = new ROSLIB.Ros();

ros.on('error', function(error)  {
  console.log(error);
});


ros.connect(ros_server);


// Publishers
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

// Subscriber
var listener = new ROSLIB.Topic({
  ros: ros,
  name: '/microROS/ultraSonido',
  messageType: 'std_msgs/msg/Int16'
});

var dist = 0;
// Subscribe
listener.subscribe(function(message) {
  console.log("Se recibió: " + message.data);
  dist = message.data;
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
})

app.get('/US', (req, res) => {
  res.send(({
    distancia: dist
  }));
})


app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})

