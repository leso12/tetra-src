// ROS Bridge 연결 설정
const ros = new ROSLIB.Ros({
    url : 'ws://192.168.137.144:9090' // PC ip
});

// Canvas 관련 변수
let mapCanvas = null; // 맵과 로봇 모두 이 캔버스에 그립니다.
let mapCtx = null; // 맵 캔버스의 2D 컨텍스트
let canvasContainer = null; // 캔버스 컨테이너 DOM 요소

// 맵 데이터 관련 변수
let mapInfo = null; // 맵의 해상도, 원점 등 정보
let mapImageData = null; // OccupancyGrid의 픽셀 데이터를 저장할 ImageData 객체
let mapImageSourceCanvas = null; // 맵 ImageData를 그릴 임시 캔버스 (OffscreenCanvas 사용 권장)
let mapImageSourceCtx = null; // 임시 캔버스의 컨텍스트

// 확대/축소 관련 변수
let currentZoomLevel = 1.0;
const zoomStep = 0.2;
const minZoomLevel = 0.5;
const maxZoomLevel = 3.0;

// 패닝(Panning) 관련 변수
let isDragging = false;
let lastMouseX = 0;
let lastMouseY = 0;
let offsetX = 0; // 캔버스 픽셀 기준의 X 오프셋 (마우스 드래그에 따라 변경)
let offsetY = 0; // 캔버스 픽셀 기준의 Y 오프셋 (마우스 드래그에 따라 변경)

// 로봇 이동 제어 관련 변수
let cmdVel = null; // ROS Twist 메시지 발행을 위한 퍼블리셔
let moveInterval = null; // 로봇 이동 명령을 주기적으로 보내기 위한 인터벌 ID

// 현재 선택된 로봇의 cmd_vel 토픽 이름과 TF 프레임 이름 (전역 변수로 선언)
let currentCmdVelTopic = '/robot1/cmd_vel'; // 초기값 설정 (필수)
let currentSelectedRobotTFFrame = '/robot1/base_link'; // 초기값 설정 (필수)

// 로봇별 위치 정보를 저장할 객체 (모든 로봇의 최신 TF를 저장)
const robotPoses = {
    robot1: { x: 0, y: 0, yaw: 0, color: 'rgba(0, 0, 255, 0.7)', status: '정지' },   // 파란색
    robot2: { x: 0, y: 0, yaw: 0, color: 'rgba(255, 0, 0, 0.7)', status: '정지' },    // 빨간색
    robot3: { x: 0, y: 0, yaw: 0, color: 'rgba(0, 255, 0, 0.7)', status: '정지' }     // 녹색
};

// --- ROSBridge 연결 상태 콜백 함수 ---
ros.on('connection', function() {
    console.log("Connected to ROSBridge!");
    if (mapCanvas && mapCtx && canvasContainer) {
        subscribeToMap();
        subscribeToTF(); // TF 구독은 모든 로봇에 대해 한 번만 설정
    } else {
        console.warn("DOM elements not ready yet. ROS subscriptions will start after DOMContentLoaded.");
    }
});

ros.on('close', function() {
    console.log("Connection to ROSBridge closed.");
    if (moveInterval) {
        clearInterval(moveInterval);
        moveInterval = null;
    }
});

ros.on('error', function(error) {
    console.error("Error connecting to ROSBridge: ", error);
});

// --- 맵 토픽 구독 함수 (nav_msgs/OccupancyGrid) ---
function subscribeToMap() {
    const mapListener = new ROSLIB.Topic({
        ros : ros,
        name : '/map',
        messageType : 'nav_msgs/OccupancyGrid'
    });

    mapListener.subscribe(function(message) {
        console.log("'/map' 메시지 도착, 맵 업데이트.");
        if (mapCanvas && mapCtx) {
            // 맵 정보가 변경되었거나, 처음 로드될 때 캔버스 크기 및 이미지 데이터 초기화
            if (!mapInfo || mapInfo.width !== message.info.width || mapInfo.height !== message.info.height) {
                mapInfo = message.info;
                try {
                    mapImageSourceCanvas = new OffscreenCanvas(mapInfo.width, mapInfo.height);
                    console.log("Using OffscreenCanvas for map image source.");
                } catch (e) {
                    mapImageSourceCanvas = document.createElement('canvas');
                    mapImageSourceCanvas.width = mapInfo.width;
                    mapImageSourceCanvas.height = mapInfo.height;
                    console.log("Using HTMLCanvasElement for map image source (OffscreenCanvas not supported).");
                }
                mapImageSourceCtx = mapImageSourceCanvas.getContext('2d');
                mapImageData = mapImageSourceCtx.createImageData(mapInfo.width, mapInfo.height);
            } else {
                mapInfo = message.info; // 맵 정보만 업데이트 (크기는 동일)
            }

            const data = mapImageData.data;

            // 맵 데이터를 픽셀 데이터로 변환
            for (let y = 0; y < mapInfo.height; y++) {
                for (let x = 0; x < mapInfo.width; x++) {
                    const mapIndex = x + (mapInfo.height - 1 - y) * mapInfo.width;
                    const cellValue = message.data[mapIndex];
                    let color = 128; // Unknown (회색)

                    if (cellValue === 0) { // Free (흰색)
                        color = 255;
                    } else if (cellValue === 100) { // Occupied (검은색)
                        color = 0;
                    }

                    const arrayIndex = (y * mapInfo.width + x) * 4;
                    data[arrayIndex + 0] = color;
                    data[arrayIndex + 1] = color;
                    data[arrayIndex + 2] = color;
                    data[arrayIndex + 3] = 255; // Alpha
                }
            }

            if (mapImageSourceCtx && mapImageData) {
                 mapImageSourceCtx.putImageData(mapImageData, 0, 0);
            } else {
                console.error("mapImageSourceCtx or mapImageData is null, cannot draw to source canvas.");
            }

            // 맵 데이터가 새로 도착했을 때 맵을 캔버스 중앙에 배치
            adjustPanOffset();

        } else {
            console.error("Error: mapCanvas 또는 mapCtx가 null입니다. DOMContentLoaded 이후에 함수가 호출되었는지 확인하세요.");
        }
    });
    console.log("Subscribed to /map topic.");
}

// --- TF (Transformations) 토픽 구독 함수 ---
function subscribeToTF() {
    console.log("subscribeToTF 함수 호출됨.");
    const tfClient = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : '/map', // 모든 로봇의 base_link를 /map 기준으로 변환
        angularThresh : 0.01,
        transThresh : 0.01,
        rate : 10.0
    });

    // 모든 로봇의 base_link 프레임에 대한 TF 구독을 설정합니다.
    const robotFrames = [
        { id: 'robot1', frame: '/robot1/base_link' },
        { id: 'robot2', frame: '/robot2/base_link' },
        { id: 'robot3', frame: '/robot3/base_link' }
    ];

    robotFrames.forEach(robot => {
        tfClient.subscribe(robot.frame, function(tf) {
            // TF 데이터는 fixedFrame (여기서는 /map) 기준으로 제공됩니다.
            const position = tf.translation;
            const orientation = tf.rotation;
            const yaw = getYawFromQuaternion(orientation);

            // 로봇별 위치 정보 객체에 TF 데이터 저장
            robotPoses[robot.id].x = position.x;
            robotPoses[robot.id].y = position.y;
            robotPoses[robot.id].yaw = yaw; // 라디안 값으로 저장 (그림 그릴 때 사용)

            updateRobotInfoTable(robot.id); // 모든 로봇의 테이블 정보 업데이트
        });
        console.log(`Subscribed to /tf topic for ${robot.frame} from ${tfClient.fixedFrame} frame.`);
    });
}

// --- 로봇 위치 정보 표를 업데이트하는 함수 ---
function updateRobotInfoTable(robotId) {
    const robot = robotPoses[robotId];
    if (robot) {
        document.getElementById(`${robotId}_x`).textContent = robot.x.toFixed(2);
        document.getElementById(`${robotId}_y`).textContent = robot.y.toFixed(2);
        document.getElementById(`${robotId}_yaw`).textContent = (robot.yaw * 180 / Math.PI).toFixed(2);

        const statusCell = document.getElementById(`${robotId}_status`);
        statusCell.textContent = robot.status; // 상태 텍스트 업데이트
        // CSS 클래스를 이용하여 색상 변경 (CSS는 이미 HTML에 정의되어 있습니다.)
        if (robot.status === '이동 중') {
            statusCell.classList.add('moving');
            statusCell.classList.remove('stopped');
        } else {
            statusCell.classList.add('stopped');
            statusCell.classList.remove('moving');
        }
    }
}

// Quaternion을 Yaw (Z축 회전) 값(라디안)으로 변환하는 함수
function getYawFromQuaternion(q) {
    return Math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

// --- 최종적으로 Canvas에 맵과 로봇을 함께 그리는 통합 함수 ---
function drawMapAndRobot() {
    //if (!mapCtx || !mapInfo || !mapImageSourceCanvas) {
    //    requestAnimationFrame(drawMapAndRobot); // 맵 정보가 없으면 다시 시도
    //    return;
    //}

    // 캔버스 초기화
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    // 맵 밖 배경색 설정 (회색)
    mapCtx.fillStyle = 'rgb(128, 128, 128)';
    mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);

    // 맵 데이터와 로봇을 그릴 때 적용할 전체 변환 저장
    mapCtx.save();

    // 패닝 및 줌 변환 적용
    mapCtx.translate(offsetX, offsetY);
    mapCtx.scale(currentZoomLevel, currentZoomLevel);

    // 1. 맵 이미지 그리기 (drawImage 사용)
    mapCtx.drawImage(mapImageSourceCanvas, 0, 0, mapInfo.width, mapInfo.height);

    // 2. 모든 로봇 아이콘 그리기
    const resolution = mapInfo.resolution;
    const originX = mapInfo.origin.position.x;
    const originY = mapInfo.origin.position.y;
    const height = mapInfo.height;

    for (const robotId in robotPoses) {
        const robot = robotPoses[robotId];
        // 로봇의 위치 데이터가 유효한지 확인
        if (robot && typeof robot.x === 'number' && typeof robot.y === 'number' && typeof robot.yaw === 'number') {
            mapCtx.save(); // 각 로봇 아이콘을 위한 변환 상태 저장

            // ROS 좌표계(m)를 캔버스 픽셀 좌표계로 변환 (Y축 반전 고려)
            const robotCanvasX = (robot.x - originX) / resolution;
            const robotCanvasY = height - ((robot.y - originY) / resolution);

            mapCtx.translate(robotCanvasX, robotCanvasY);
            mapCtx.rotate(-robot.yaw); // ROS Yaw와 캔버스 회전 방향 일치

            // 로봇 아이콘 (삼각형 모양)
            const robotSize = 7; // 로봇 아이콘 크기
            mapCtx.beginPath();
            mapCtx.moveTo(robotSize, 0); // 로봇의 앞부분
            mapCtx.lineTo(-robotSize * 0.7, -robotSize * 0.7); // 왼쪽 뒷부분
            mapCtx.lineTo(-robotSize * 0.7, robotSize * 0.7); // 오른쪽 뒷부분
            mapCtx.closePath();

            mapCtx.fillStyle = robot.color; // 각 로봇에 설정된 색상 사용
            mapCtx.fill();
            mapCtx.strokeStyle = 'darkblue'; // 테두리 색상
            mapCtx.lineWidth = 1.5;
            mapCtx.stroke();

            // 로봇 ID 표시 (선택 사항)
            mapCtx.fillStyle = 'black';
            mapCtx.font = 'bold 8px Arial';
            mapCtx.textAlign = 'center';
            mapCtx.textBaseline = 'middle';
            mapCtx.fillText(robotId.replace('robot', 'R'), 0, robotSize + 5); // 예: robot1 -> R1

            mapCtx.restore(); // 로봇 아이콘 변환 상태 복원
        }
    }

    mapCtx.restore(); // 전체 변환 상태 복원
    requestAnimationFrame(drawMapAndRobot); // 애니메이션 루프: 다음 프레임을 요청하여 지속적으로 그립니다.
}


// --- 로봇 이동 제어 함수 ---
function startMove(direction) {
    let linear_x = 0;
    let angular_z = 0;

    const linearSpeed = 0.2;
    const angularSpeed = 0.2;

    switch (direction) {
        case 'forward': linear_x = linearSpeed; break;
        case 'backward': linear_x = -linearSpeed; break;
        case 'left': angular_z = angularSpeed; break;
        case 'right': angular_z = -angularSpeed; break;
    }

    const twist = new ROSLIB.Message({
        linear: { x: linear_x, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular_z }
    });

    if (!cmdVel || cmdVel.name !== currentCmdVelTopic) {
        cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : currentCmdVelTopic, // 선택된 로봇의 토픽 이름 사용
            messageType : 'geometry_msgs/Twist'
        });
        console.log(`cmd_vel publisher updated to: ${currentCmdVelTopic}`);
    }

    // 선택된 로봇의 상태를 '이동 중'으로 업데이트
    const selectedRobotId = document.getElementById('robotSelector').selectedOptions[0].dataset.tfFrame.split('/')[1];
    if (robotPoses[selectedRobotId]) {
        robotPoses[selectedRobotId].status = '이동 중';
        updateRobotInfoTable(selectedRobotId); // 테이블 즉시 업데이트
    }

    if (moveInterval) {
        clearInterval(moveInterval);
    }

    moveInterval = setInterval(() => {
        cmdVel.publish(twist);
        console.log(`Moving ${currentCmdVelTopic}: Linear ${twist.linear.x.toFixed(2)}, Angular ${twist.angular.z.toFixed(2)}`);
    }, 100);
}

function stopMove() {
    if (moveInterval) {
        clearInterval(moveInterval);
        moveInterval = null;
    }
    // 선택된 로봇의 상태를 '정지'로 업데이트
    const selectedRobotId = document.getElementById('robotSelector').selectedOptions[0].dataset.tfFrame.split('/')[1];
    if (robotPoses[selectedRobotId]) {
        robotPoses[selectedRobotId].status = '정지';
        updateRobotInfoTable(selectedRobotId); // 테이블 즉시 업데이트
    }
    sendStopCommand();
}

function sendStopCommand() {
    const stopTwist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
    });

    if (!cmdVel || cmdVel.name !== currentCmdVelTopic) {
        cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : currentCmdVelTopic, // 선택된 로봇의 토픽 이름 사용
            messageType : 'geometry_msgs/Twist'
        });
        console.log(`Stop command publisher updated to: ${currentCmdVelTopic}`);
    }
    cmdVel.publish(stopTwist);
    console.log(`Stop command sent to ${currentCmdVelTopic}.`);
}

// --- 확대/축소 (Zoom) 관련 함수 ---
function setZoom(level) {
    currentZoomLevel = Math.max(minZoomLevel, Math.min(maxZoomLevel, level));
    adjustPanOffset();
    console.log("Zoom level set to:", currentZoomLevel);
}

function zoomIn() {
    setZoom(currentZoomLevel + zoomStep);
}

function zoomOut() {
    setZoom(currentZoomLevel - zoomStep);
}

// --- 패닝(Panning) 함수들 ---
function startDragging(e) {
    if (e.button === 0) { // 좌클릭만 처리
        isDragging = true;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;
        canvasContainer.style.cursor = 'grabbing';
    }
}

function dragMap(e) {
    if (!isDragging || !mapInfo) return;

    const dx = e.clientX - lastMouseX;
    const dy = e.clientY - lastMouseY;

    offsetX += dx;
    offsetY += dy;

    lastMouseX = e.clientX;
    lastMouseY = e.clientY;

    // 패닝 중에는 애니메이션 프레임 요청을 하지 않고, 바로 그립니다.
    // drawMapAndRobot()는 requestAnimationFrame으로 계속 호출되므로 여기서는 불필요
}

function stopDragging() {
    isDragging = false;
    canvasContainer.style.cursor = 'grab';
}

// 맵이 로드되거나 줌 레벨이 변경될 때 맵을 캔버스 중앙으로 정렬하는 함수
function adjustPanOffset() {
    if (!mapInfo || !mapImageSourceCanvas || !mapCanvas) return;

    const scaledMapWidth = mapInfo.width * currentZoomLevel;
    const scaledMapHeight = mapInfo.height * currentZoomLevel;

    const canvasWidth = mapCanvas.width;
    const canvasHeight = mapCanvas.height;

    // 맵을 캔버스 중앙에 오도록 오프셋 계산
    offsetX = (canvasWidth - scaledMapWidth) / 2;
    offsetY = (canvasHeight - scaledMapHeight) / 2;

    // drawMapAndRobot()는 requestAnimationFrame으로 계속 호출되므로 여기서는 불필요
}

// 캔버스 크기를 컨테이너 크기에 맞게 조정
function resizeCanvasToContainer() {
    if (!canvasContainer || !mapCanvas) return;

    const width = canvasContainer.clientWidth;
    const height = canvasContainer.clientHeight;

    mapCanvas.width = width;
    mapCanvas.height = height;

    adjustPanOffset();
}

// --- 마우스 휠 이벤트 핸들러 ---
function handleMouseWheel(e) {
    e.preventDefault(); // 기본 스크롤 동작 방지

    if (e.deltaY < 0) { // 휠을 위로 굴리면 줌인
        zoomIn();
    } else { // 휠을 아래로 굴리면 줌아웃
        zoomOut();
    }
}

// --- 로봇 선택 변경 핸들러 ---
function updateSelectedRobot() {
    const selector = document.getElementById('robotSelector');
    currentCmdVelTopic = selector.value;
    currentSelectedRobotTFFrame = selector.options[selector.selectedIndex].dataset.tfFrame;

    console.log("선택된 로봇 토픽:", currentCmdVelTopic);
    console.log("선택된 로봇 TF 프레임:", currentSelectedRobotTFFrame);
}

// --- DOM 콘텐츠가 모두 로드된 후 실행될 함수 ---
document.addEventListener('DOMContentLoaded', function() {
    mapCanvas = document.getElementById('mapCanvas');
    canvasContainer = document.querySelector('.canvas-container');

    if (mapCanvas) mapCtx = mapCanvas.getContext('2d');

    if (!mapCanvas || !mapCtx || !canvasContainer) {
        console.error("ERROR: 필수 DOM 요소가 HTML에 없습니다! ID를 확인하세요.");
        return;
    }

    resizeCanvasToContainer();

    if (ros.isConnected) {
        subscribeToMap();
        subscribeToTF();
    }

    canvasContainer.addEventListener('mousedown', startDragging);
    canvasContainer.addEventListener('mousemove', dragMap);
    canvasContainer.addEventListener('mouseup', stopDragging);
    canvasContainer.addEventListener('mouseleave', stopDragging);
    canvasContainer.addEventListener('wheel', handleMouseWheel);

    window.addEventListener('resize', resizeCanvasToContainer);

    const robotSelector = document.getElementById('robotSelector');
    if (robotSelector) {
        robotSelector.addEventListener('change', updateSelectedRobot);
        // 초기 선택된 로봇에 대한 정보 업데이트 (페이지 로드 시)
        updateSelectedRobot();
    } else {
        console.warn("로봇 선택 셀렉트 박스 (ID: 'robotSelector')를 찾을 수 없습니다.");
    }

    // 맵 애니메이션 루프 시작
    requestAnimationFrame(drawMapAndRobot);
});