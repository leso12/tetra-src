// ROS Bridge 연결 설정 (index.html의 것과 동일)
const ros = new ROSLIB.Ros({
    url : 'ws://61.250.168.131:9090' // PC ip
});

let mapCanvas = null;
let statusDisplay = null; // 맵핑 상태를 표시할 엘리먼트
let mapInfo = null;
let mapImageData = null;

// 확대/축소 관련 변수 (index.html의 robot_control.js와 동일)
let currentZoomLevel = 1.0;
const zoomStep = 0.2;
const minZoomLevel = 0.5;
const maxZoomLevel = 3.0;
let canvasContainer = null;

// ROSBridge 연결 상태 콜백 함수
ros.on('connection', function() {
    console.log("Connected to ROSBridge!");
    if (mapCanvas && statusDisplay) {
        subscribeToMap(); // 맵핑 중인 맵 구독
        subscribeToTF(); // 로봇 위치 구독 (맵핑 중 로봇 위치 표시 위함)
        statusDisplay.textContent = "맵핑 상태: ROS 연결됨, 맵 대기 중...";
    } else {
        console.warn("DOM elements not ready yet. Subscriptions will start after DOMContentLoaded.");
    }
});

ros.on('close', function() {
    console.log("Connection to ROSBridge closed.");
    statusDisplay.textContent = "맵핑 상태: ROS 연결 끊김.";
});

ros.on('error', function(error) {
    console.error("Error connecting to ROSBridge: ", error);
    statusDisplay.textContent = "맵핑 상태: 연결 에러!";
});

// 맵 토픽 구독 함수 (nav_msgs/OccupancyGrid) - index.html의 subscribeToMap과 거의 동일
function subscribeToMap() {
    const mapListener = new ROSLIB.Topic({
        ros : ros,
        name : '/map',
        messageType : 'nav_msgs/OccupancyGrid'
    });

    mapListener.subscribe(function(message) {
        console.log("'/map' 메시지 도착, 맵 업데이트.");
        if (mapCanvas && mapCanvas.getContext) {
            const ctx = mapCanvas.getContext('2d');
            mapInfo = message.info;

            const width = mapInfo.width;
            const height = mapInfo.height;

            mapCanvas.width = width;
            mapCanvas.height = height;

            // 컨테이너의 초기 표시 크기를 맵 해상도에 비례하여 설정 (index.html과 동일)
            const maxDim = Math.max(width, height);
            const initialContainerSize = 700; // 기준이 되는 픽셀 크기
            canvasContainer.style.width = `${initialContainerSize}px`;
            canvasContainer.style.height = `${initialContainerSize * (height / width)}px`;

            const imageData = ctx.createImageData(width, height);
            const data = imageData.data;

            for (let y = 0; y < height; y++) {
                for (let x = 0; x < width; x++) {
                    const mapIndex = x + (height - 1 - y) * width;
                    const cellValue = message.data[mapIndex];
                    let color = 128; // Unknown (회색)
                    if (cellValue === 0) { color = 255; } else if (cellValue === 100) { color = 0; }
                    const arrayIndex = (y * width + x) * 4;
                    data[arrayIndex + 0] = color;
                    data[arrayIndex + 1] = color;
                    data[arrayIndex + 2] = color;
                    data[arrayIndex + 3] = 255;
                }
            }
            mapImageData = imageData;

            drawMapAndRobot(ctx); // 맵과 로봇을 함께 그립니다.
            statusDisplay.textContent = `맵핑 상태: 맵 업데이트 중 (${width}x${height})`;
        } else {
            console.error("Error: mapCanvas 또는 context가 null입니다. DOMContentLoaded 이후에 함수가 호출되었는지 확인하세요.");
        }
    });
    console.log("Subscribed to /map topic.");
}


// 로봇의 현재 위치와 방향을 저장할 변수 (TF 구독을 위해 필요)
let robotPose = {
    x: 0,
    y: 0,
    yaw: 0
};

// TF (Transformations) 토픽 구독 함수 (index.html의 것과 동일)
function subscribeToTF() {
    const tfClient = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : '/map',
        angularThresh : 0.01,
        transThresh : 0.01,
        rate : 10.0
    });

    tfClient.subscribe('/base_link', function(tf) {
        const position = tf.translation;
        const orientation = tf.rotation;

        const yaw = getYawFromQuaternion(orientation);

        robotPose.x = position.x;
        robotPose.y = position.y;
        robotPose.yaw = yaw;

        // 맵 정보와 맵 이미지가 모두 로드된 경우에만 로봇을 그립니다.
        if (mapCanvas && mapInfo && mapImageData) {
            drawMapAndRobot(mapCanvas.getContext('2d'));
        }
    });
    console.log("Subscribed to /tf topic for /base_link from /map frame.");
}

// Quaternion to Yaw (Z-axis rotation) 함수 - 동일
function getYawFromQuaternion(q) {
    return Math.atan2(2 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// 최종적으로 Canvas에 맵과 로봇을 그리는 통합 함수 - 동일
function drawMapAndRobot(ctx) {
    if (!mapCanvas || !mapInfo || !mapImageData) return;

    const resolution = mapInfo.resolution;
    const originX = mapInfo.origin.position.x;
    const originY = mapInfo.origin.position.y;
    const width = mapInfo.width;
    const height = mapInfo.height;

    // 맵 데이터를 Canvas에 그립니다.
    ctx.putImageData(mapImageData, 0, 0);

    // 로봇 위치를 Canvas 픽셀 좌표로 변환
    const robotCanvasX = (robotPose.x - originX) / resolution;
    const robotCanvasY = height - ((robotPose.y - originY) / resolution);

    ctx.save();
    ctx.translate(robotCanvasX, robotCanvasY);
    ctx.rotate(-robotPose.yaw);

    // 로봇 아이콘 (직사각형 + 삼각형) - 동일
    const rectWidth = 8;
    const rectHeight = 12;
    const triangleSize = 8;

    // 1. 직사각형 그리기
    ctx.fillStyle = 'rgba(0, 0, 255, 0.7)';
    ctx.fillRect(-rectWidth / 2, -rectHeight / 2, rectWidth, rectHeight);
    ctx.strokeStyle = 'darkblue';
    ctx.lineWidth = 1;
    ctx.strokeRect(-rectWidth / 2, -rectHeight / 2, rectWidth, rectHeight);

    // 2. 삼각형 그리기 (좌우 뒤집기 - 오른쪽으로 뾰족하게)
    ctx.beginPath();
    ctx.moveTo(rectWidth / 2, 0); // 삼각형의 가장 오른쪽 꼭짓점
    ctx.lineTo(-rectWidth / 2, -triangleSize / 2); // 직사각형의 왼쪽 변에 닿는 윗점
    ctx.lineTo(-rectWidth / 2, triangleSize / 2); // 직사각형의 왼쪽 변에 닿는 아랫점
    ctx.closePath();

    ctx.fillStyle = 'white';
    ctx.fill();
    ctx.strokeStyle = 'darkblue';
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.restore();
}

// 맵 저장 함수 (미구현)
function saveMap() {
    // ROS 서비스 호출을 통해 맵 저장 (map_saver)
    // 이 부분은 ROS 서비스 클라이언트 로직을 추가해야 합니다.
    // 예: rosrun map_server map_saver -f /path/to/my_map
    // ROS 서비스 호출 예시:
    // const mapSaverClient = new ROSLIB.Service({
    //     ros : ros,
    //     name : '/map_saver/save_map', // map_saver 패키지의 서비스 이름
    //     serviceType : 'map_msgs/SaveMap' // 적절한 서비스 타입
    // });
    // const request = new ROSLIB.ServiceRequest({
    //     filename : '/home/ani/my_robot_map' // 저장할 경로 및 파일명
    // });
    // mapSaverClient.callService(request, function(result) {
    //     if (result.success) {
    //         statusDisplay.textContent = "맵 저장 성공!";
    //     } else {
    //         statusDisplay.textContent = "맵 저장 실패!";
    //     }
    // });
    statusDisplay.textContent = "맵 저장 기능은 아직 구현되지 않았습니다. 터미널에서 'rosrun map_server map_saver -f <경로>'를 사용하세요.";
    console.log("맵 저장 버튼 클릭됨!");
}

// 확대/축소 관련 함수 (index.html의 것과 동일)
function setZoom(level) {
    currentZoomLevel = Math.max(minZoomLevel, Math.min(maxZoomLevel, level));

    const baseWidth = mapCanvas.width;
    const baseHeight = mapCanvas.height;

    if (baseWidth === 0 || baseHeight === 0) {
        console.warn("Map dimensions not set yet, cannot set zoom.");
        return;
    }

    const initialDisplayWidth = 700; // HTML CSS에서 설정한 초기 컨테이너 width와 일치
    const scaleFactor = initialDisplayWidth / baseWidth;

    const newDisplayWidth = baseWidth * scaleFactor * currentZoomLevel;
    const newDisplayHeight = baseHeight * scaleFactor * currentZoomLevel;

    canvasContainer.style.width = `${newDisplayWidth}px`;
    canvasContainer.style.height = `${newDisplayHeight}px`;

    drawMapAndRobot(mapCanvas.getContext('2d')); // 줌 변경 시 맵 다시 그리기
    console.log("Zoom level set to:", currentZoomLevel);
}

function zoomIn() {
    setZoom(currentZoomLevel + zoomStep);
}

function zoomOut() {
    setZoom(currentZoomLevel - zoomStep);
}


// DOM 콘텐츠가 모두 로드된 후 실행될 함수
document.addEventListener('DOMContentLoaded', function() {
    mapCanvas = document.getElementById('mapCanvas');
    statusDisplay = document.getElementById('status_display');
    canvasContainer = document.querySelector('.canvas-container');

    if (!mapCanvas || !statusDisplay || !canvasContainer) {
        console.error("ERROR: 필수 DOM 요소가 mapping.html에 없습니다!");
        return;
    }

    if (ros.isConnected) {
        subscribeToMap();
        subscribeToTF();
    }
});