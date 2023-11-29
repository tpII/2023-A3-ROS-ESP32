const express = require('express')
const path = require('path');
const ROSLIB = require('roslib')
const app = express()
const port = 3000

var ros_server = 'ws://localhost:9090';
var ros = new ROSLIB.Ros();
var led_state = 0;
ros.connect(ros_server);

var state = new ROSLIB.Topic({
    ros : ros,
    name : '/microROS/string',
    messageType : 'std_msgs/msg/String'
  });

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '/index.html'));
})

app.use(express.json());
app.use(express.urlencoded({ extended: true }));

app.post('/STRING', (req, res) => {

    var x = parseFloat(req.body.x);
    var y = parseFloat(req.body.y);
    var led_temp = parseInt(req.body.data);

    if (isNaN(x)){
      x = 0;
    }

    if (isNaN(y)) {
      y = 0;
    }

    if (!isNaN(led_temp)) {
      led_state = led_temp;
    }

    var ults = 10;

    var str = "(" + x + ", " + y + ")" + " - " + led_state + " - " + ults;
    console.log(str);

    var stringAction = new ROSLIB.Message({
        data : str
      });

    state.publish(stringAction);
    //console.log(req.body);
    res.redirect('/');
})

app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})

