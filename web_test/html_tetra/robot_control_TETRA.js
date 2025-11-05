const ros = new ROSLIB.Ros({
    url : 'ws://192.168.33.2:9090' // TETRA ip
});

let mapCanvas = null;
let locationDisplay = null;
let saveLocationButton = null;
const savedLocationsKey = 'savedRobotLocations';
let moveInterval;

ros.on('connection', function() {
    console.log("Connected to ROSBridge!");
    subscribeToOdom();
    subscribeToMap();
});

ros.on('close', function() {
    console.log("Connection to ROSBridge closed.");
});

ros.on('error', function(error) {
    console.error("Error connecting to ROSBridge: ", error);
});

function subscribeToOdom() {
    const odomListener = new ROSLIB.Topic({
        ros : ros,
        name : '/TETRA_NS/odom',
        messageType : 'nav_msgs/Odometry'
    });

    odomListener.subscribe(function(message) {
        console.log("'/TETRA_NS/odom' 메시지 도착:", message);
        const position = message.pose.pose.position;
        const orientation = message.pose.pose.orientation;
        const yaw = Math.atan2(2 * (orientation.x * orientation.w + orientation.y * orientation.z), 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y));
        const yawDegrees = yaw * 180 / Math.PI;
        console.log("Yaw (degrees):", yawDegrees);
        if (locationDisplay) {
            locationDisplay.textContent = `현재 위치: X=${position.x.toFixed(2)}, Y=${position.y.toFixed(2)}, Z=${position.z.toFixed(2)}, Yaw=${yawDegrees.toFixed(5)}`;
        } else {
            console.log("Error: locationDisplay 요소가 null입니다.");
        }
    });
    console.log("Subscribed to /TETRA_NS/odom topic.");
}

function subscribeToMap() {
    const mapListener = new ROSLIB.Topic({
        ros : ros,
        name : '/TETRA_NS/map',
        messageType : 'nav_msgs/OccupancyGrid'
    });

    mapListener.subscribe(function(message) {
        console.log('Received map:', message);
        const mapData = message.data;
        const mapInfo = message.info;
        if (mapCanvas && mapCanvas.getContext) {
            const ctx = mapCanvas.getContext('2d');
            const width = mapInfo.width;
            const height = mapInfo.height;

            mapCanvas.width = width;
            mapCanvas.height = height;

            const imageData = ctx.createImageData(width, height);
            for (let y = 0; y < height; y++) {
                for (let x = 0; x < width; x++) {
                    const index = y * width + x;
                    const value = mapData[index];
                    let color;
                    if (value === -1) { // Unknown
                        color = [200, 200, 200, 255]; // Gray
                    } else if (value === 0) { // Free space
                        color = [255, 255, 255, 255]; // White
                    } else { // Occupied (0-100 probability)
                        const grayScale = Math.round((100 - value) * 2.55);
                        color = [grayScale, grayScale, grayScale, 255]; // Gray scale
                    }
                    imageData.data[(y * width + x) * 4] = color[0];
                    imageData.data[(y * width + x) * 4 + 1] = color[1];
                    imageData.data[(y * width + x) * 4 + 2] = color[2];
                    imageData.data[(y * width + x) * 4 + 3] = color[3];
                }
            }
            ctx.putImageData(imageData, 0, 0);
            console.log('Received and rendered map.');
        } else {
            console.log("Error: mapCanvas 요소가 null이거나 getContext를 사용할 수 없습니다.");
        }
    });
    console.log("Subscribed to /TETRA_NS/map topic.");
}

function startMove(direction) {
    let linear_x = 0;
    let angular_z = 0;

    switch (direction) {
        case 'forward': linear_x = 0.2; break;
        case 'backward': linear_x = -0.2; break;
        case 'left': angular_z = 0.3; break;
        case 'right': angular_z = -0.3; break;
    }

    const twist = new ROSLIB.Message({
        linear: { x: linear_x, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular_z }
    });

    const cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/TETRA_NS/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    moveInterval = setInterval(() => {
        cmdVel.publish(twist);
    }, 100);
}

function stopMove() {
    clearInterval(moveInterval);
    sendStopCommand();
}

function sendStopCommand() {
    const stopTwist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
    });

    const cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/TETRA_NS/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    cmdVel.publish(stopTwist);
}

function setupSaveLocationButton() {
    if (saveLocationButton) {
        saveLocationButton.addEventListener('click', function() {
            if (locationDisplay && locationDisplay.textContent !== '현재 위치: 알 수 없음') {
                const currentLocation = locationDisplay.textContent;
                let savedLocations = localStorage.getItem(savedLocationsKey);
                savedLocations = savedLocations ? JSON.parse(savedLocations) : [];
                savedLocations.push(currentLocation);
                localStorage.setItem(savedLocationsKey, JSON.stringify(savedLocations));
                alert('현재 위치를 저장했습니다.');
                console.log('저장된 위치:', savedLocations);
            } else {
                alert('현재 위치 정보를 가져올 수 없습니다.');
            }
        });
    } else {
        console.log("Error: saveLocationButton 요소가 null입니다.");
    }
}

function displaySavedLocations() {
    if (typeof localStorage !== 'undefined') {
        const savedLocations = localStorage.getItem(savedLocationsKey);
        if (savedLocations) {
            const locationsArray = JSON.parse(savedLocations);
            console.log('불러온 저장된 위치:', locationsArray);
        }
    } else {
        console.log("localStorage is not available.");
    }
}

document.addEventListener('DOMContentLoaded', function() {
    mapCanvas = document.getElementById('mapCanvas');
    locationDisplay = document.getElementById('location_display');
    saveLocationButton = document.getElementById('save_location');
    setupSaveLocationButton();
    displaySavedLocations();
});