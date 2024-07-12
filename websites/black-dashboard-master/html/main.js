var app = new Vue({
  el: '#app',
  computed: {
    ws_address: function() {
      return `${this.rosbridge_address}`
    },
  },
  data: {
    connected: false,
    ros: null,
    logs: [],
    loading: false,
    topic: null,
    message: null,
    rosbridge_address: '',
    port: '9090',
    service_busy: false,
    service_response: '',
  },
  methods: {
    connect_rosbridge: function() {
      console.log('ROS Bridge function triggered.');
      this.loading = true
      this.ros = new ROSLIB.Ros({
        url: this.ws_address
      })
      this.ros.on('connection', () => {
        console.log('Connected to websocket server.');
        this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
        this.connected = true
        this.loading = false
        this.setCamera()
        this.setImageProcessed()
      })
      this.ros.on('error', (error) => {
        console.log('Failed to connect to websocket server.');
        this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
      })
      this.ros.on('close', () => {
        this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
        this.connected = false
        this.loading = false
        document.getElementById('divCamera').innerHTML = ''
        document.getElementById('divImageProcessed').innerHTML = ''
      })
    },
    disconnect: function() {
      this.ros.close()
    },
    setCamera: function() {
      let without_wss = this.rosbridge_address.split('wss://')[1]
      let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
      let host = domain + '/cameras'
      let viewer = new MJPEGCANVAS.Viewer({
        divID: 'divCamera',
        host: host,
        width: 384,
        height: 216,
        topic: '/demo_cam/camera1/image_raw',
        ssl: true,
      })
      viewer.canvas.classList.add('camera-image');
    },
    setImageProcessed: function() {
      let without_wss = this.rosbridge_address.split('wss://')[1]
      let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
      let host = domain + '/cameras'
      let viewer = new MJPEGCANVAS.Viewer({
        divID: 'divImageProcessed',
        host: host,
        width: 384,
        height: 216,
        topic: '/image_processed',
        ssl: true,
      })
      viewer.canvas.classList.add('camera-image');
    },
    callNewGameService: function() {
        // Service is busy
        this.service_busy = true
        this.service_response = ''
        // Define the service to be called
        let service = new ROSLIB.Service({
            ros: this.ros,
            name: '/start_new_game',
            serviceType: 'std_srvs/srv/Trigger',
        })
        // Define the request
        let request = new ROSLIB.ServiceRequest({})
        // Define a callback
        service.callService(request, (result) => {
            this.service_busy = false
            this.service_response = JSON.stringify(result)
        }, (error) => {
            this.service_busy = false
            console.error(error)
        })
    },
    callNextMoveService: function() {
        // Service is busy
        this.service_busy = true
        this.service_response = ''
        // Define the service to be called
        let service = new ROSLIB.Service({
            ros: this.ros,
            name: '/ask_for_next_move',
            serviceType: 'moveit_planning/srv/MakeMove',
        })
        // Define the request
        let request = new ROSLIB.ServiceRequest({})
        // Define a callback
        service.callService(request, (result) => {
            this.service_busy = false
            this.service_response = JSON.stringify(result)
        }, (error) => {
            this.service_busy = false
            console.error(error)
        })
    },
    callNextManualService: function(inputValue) {
        // Service is busy
        this.service_busy = true
        this.service_response = ''
        // Define the service to be called
        let service = new ROSLIB.Service({
            ros: this.ros,
            name: '/ask_for_next_move',
            serviceType: 'moveit_planning/srv/MakeMove',
        })
        // Define the request
        let request = new ROSLIB.ServiceRequest({
            manual: true,
            command: inputValue})
        // Define a callback
        service.callService(request, (result) => {
            this.service_busy = false
            this.service_response = JSON.stringify(result)
        }, (error) => {
            this.service_busy = false
            console.error(error)
        })
    },
  },
  mounted() {
  },
})
