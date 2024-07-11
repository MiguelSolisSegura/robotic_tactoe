'use strict';

const e = React.createElement;

let ros = null

class AppComponent extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
        robotState: '-'
        }
    }
    name = 'React.js component'
    
    connectToRosbridge = () => {
        try {
            ros = new ROSLIB.Ros({
                url: document.getElementById('rosbridge_address').value
            })
        } catch (ex) {
            console.log(ex)
            //
        }
        ros.on('connection', function () {
            console.log('Connected to websocket server.')
        })
        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error)
        })
        ros.on('close', function () {
            console.log('Connection to websocket server closed.')
        })
    }
    robotCircles = () => {
        let topic = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        })
        let msg = new ROSLIB.Message({
            linear: { x: 0.5 },
            angular: { z: 0.5 },
        })
        topic.publish(msg)
        this.setState({
            robotState: 'running in circles...'
        })
    }

    render() {
        return (
            <div>
                <div>
                    <h2>Hello from {this.name}</h2>
                </div>

                <div>
                    <label>Enter your rosbridge address</label>
                    <br />
                    <input type="text" id="rosbridge_address" />
                    <br />
                    <button onClick={this.connectToRosbridge}>Connect</button>
                </div>

                <div>
                    <h3>Robot actions</h3>
                    <p>Robot is {this.state.robotState}</p>
                    <button onClick={this.robotCircles}>Run in circles</button>
                    <br />
                </div>
            </div>
        )
    }
}

const appContainer = document.querySelector('#app')
ReactDOM.render(e(AppComponent), appContainer)