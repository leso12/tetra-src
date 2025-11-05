// ROS Bridge 연결 설정
const ros = new ROSLIB.Ros({
    url : 'ws://61.250.168.131:9090' // PC ip
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

// 로봇 위치 및 자세 관련 변수
let robotPose = {
    x: 0,
    y: 0,
    yaw: 0
};
let locationDisplay = null; // 위치 표시용 DOM 엘리먼트

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

// --- ROSBridge 연결 상태 콜백 함수 ---
ros.on('connection', function() {
    console.log("Connected to ROSBridge!");
    if (mapCanvas && mapCtx && locationDisplay && canvasContainer) {
        subscribeToOdom();
        subscribeToMap();
        subscribeToTF();
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
    if (locationDisplay) {
        locationDisplay.textContent = "현재 위치: ROS 연결 끊김.";
    }
});

ros.on('error', function(error) {
    console.error("Error connecting to ROSBridge: ", error);
    if (locationDisplay) {
        locationDisplay.textContent = "현재 위치: 연결 에러!";
    }
});

// --- 오도메트리 토픽 구독 함수 (nav_msgs/Odometry) ---
function subscribeToOdom() {
    const odomListener = new ROSLIB.Topic({
        ros : ros,
        name : '/odom',
        messageType : 'nav_msgs/Odometry'
    });

    odomListener.subscribe(function(message) {
        const position = message.pose.pose.position;
        const orientation = message.pose.pose.orientation;
        const yaw = getYawFromQuaternion(orientation);
        const yawDegrees = yaw * 180 / Math.PI;

        if (locationDisplay) {
            locationDisplay.textContent = `현재 위치: X=${position.x.toFixed(2)}, Y=${position.y.toFixed(2)}, Yaw=${yawDegrees.toFixed(2)}°`;
        }
    });
    console.log("Subscribed to /odom topic.");
}

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
            // 맵 정보가 처음 설정될 때만 캔버스의 내부 픽셀 해상도 설정
            // 그리고 mapImageSourceCanvas를 처음 생성합니다.
            if (!mapInfo || mapInfo.width !== message.info.width || mapInfo.height !== message.info.height) {
                mapInfo = message.info;
                // 캔버스 크기를 맵 해상도에 맞게 조정 (픽셀 단위)
                // 이 캔버스는 실제 보이는 크기가 아니라 내부 해상도를 의미합니다.
                mapCanvas.width = mapInfo.width;
                mapCanvas.height = mapInfo.height;

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
                // 맵 정보만 업데이트 (크기가 동일하면 ImageData는 재생성할 필요 없음)
                mapInfo = message.info;
            }

            // 맵 이미지 데이터의 'data' 배열만 업데이트합니다.
            const data = mapImageData.data;

            for (let y = 0; y < mapInfo.height; y++) {
                for (let x = 0; x < mapInfo.width; x++) {
                    // ROS OccupancyGrid는 좌하단이 원점, Y축이 위로 증가
                    // 캔버스 ImageData는 좌상단이 원점, Y축이 아래로 증가
                    // 따라서 Y축을 반전시켜야 합니다.
                    const mapIndex = x + (mapInfo.height - 1 - y) * mapInfo.width; // ROS 맵 데이터 인덱스
                    const cellValue = message.data[mapIndex];
                    let color = 128; // Unknown (회색)

                    if (cellValue === 0) { // Free (흰색)
                        color = 255;
                    } else if (cellValue === 100) { // Occupied (검은색)
                        color = 0;
                    }

                    // 캔버스 ImageData에 직접 픽셀 데이터 쓰기
                    const arrayIndex = (y * mapInfo.width + x) * 4;
                    data[arrayIndex + 0] = color;
                    data[arrayIndex + 1] = color;
                    data[arrayIndex + 2] = color;
                    data[arrayIndex + 3] = 255; // Alpha
                }
            }

            // 업데이트된 ImageData를 임시 캔버스에 그립니다.
            if (mapImageSourceCtx && mapImageData) {
                 mapImageSourceCtx.putImageData(mapImageData, 0, 0);
            } else {
                console.error("mapImageSourceCtx or mapImageData is null, cannot draw to source canvas.");
            }

            // 맵 데이터가 새로 도착했으므로, 맵과 로봇을 함께 다시 그립니다.
            adjustPanOffset(); // 맵 정보가 업데이트될 때 패닝 제한 재조정
            drawMapAndRobot();

        } else {
            console.error("Error: mapCanvas 또는 mapCtx가 null입니다. DOMContentLoaded 이후에 함수가 호출되었는지 확인하세요.");
        }
    });
    console.log("Subscribed to /map topic.");
}

// --- TF (Transformations) 토픽 구독 함수 ---
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

        // TF 업데이트 시에도 맵과 로봇을 함께 그립니다.
        if (mapCanvas && mapInfo && mapImageSourceCanvas) {
            drawMapAndRobot();
        }
    });
    console.log("Subscribed to /tf topic for /base_link from /map frame.");
}

// Quaternion을 Yaw (Z축 회전) 값(라디안)으로 변환하는 함수
function getYawFromQuaternion(q) {
    return Math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

// --- 최종적으로 Canvas에 맵과 로봇을 함께 그리는 통합 함수 ---
function drawMapAndRobot() {
    if (!mapCtx || !mapInfo || !mapImageSourceCanvas) return;

    // 캔버스 초기화
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    mapCtx.fillStyle = 'rgb(128,128,128)'; // 맵 밖 배경색 설정
    mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height); // 전체 캔버스 채우기

    // 맵 데이터와 로봇을 그릴 때 적용할 전체 변환 저장
    mapCtx.save();

    // 패닝 및 줌 변환 적용
    // translate는 scale 이전에 적용되어야 합니다.
    mapCtx.translate(offsetX, offsetY);
    mapCtx.scale(currentZoomLevel, currentZoomLevel);


    // 1. 맵 이미지 그리기 (drawImage 사용)
    mapCtx.drawImage(mapImageSourceCanvas, 0, 0, mapInfo.width, mapInfo.height);


    // 2. 로봇 아이콘 그리기
    if (robotPose) {
        mapCtx.save(); // 로봇 아이콘만을 위한 변환을 위해 상태 다시 저장

        const resolution = mapInfo.resolution;
        const originX = mapInfo.origin.position.x;
        const originY = mapInfo.origin.position.y;
        const width = mapInfo.width;
        const height = mapInfo.height;

        // ROS 좌표계(m)를 캔버스 픽셀 좌표계로 변환 (Y축 반전 고려)
        const robotCanvasX = (robotPose.x - originX) / resolution;
        const robotCanvasY = height - ((robotPose.y - originY) / resolution);


        mapCtx.translate(robotCanvasX, robotCanvasY);
        mapCtx.rotate(-robotPose.yaw); // ROS Yaw와 캔버스 회전 방향 일치

        // 로봇 아이콘 (삼각형 모양)
        const robotSize = 7;
        mapCtx.beginPath();
        mapCtx.moveTo(robotSize, 0);
        mapCtx.lineTo(-robotSize * 0.7, -robotSize * 0.7);
        mapCtx.lineTo(-robotSize * 0.7, robotSize * 0.7);
        mapCtx.closePath();

        mapCtx.fillStyle = 'rgba(0, 0, 255, 0.7)';
        mapCtx.fill();
        mapCtx.strokeStyle = 'darkblue';
        mapCtx.lineWidth = 2;
        mapCtx.stroke();

        mapCtx.restore(); // 로봇 아이콘 변환 상태 복원
    }

    mapCtx.restore(); // 전체 변환 상태 복원
}


// --- 로봇 이동 제어 함수 (이전과 동일) ---
function startMove(direction) {
    let linear_x = 0;
    let angular_z = 0;

    const linearSpeed = 0.2;
    const angularSpeed = 0.5;

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

    if (!cmdVel) {
        cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/Twist'
        });
    }

    if (moveInterval) {
        clearInterval(moveInterval);
    }

    moveInterval = setInterval(() => {
        cmdVel.publish(twist);
        console.log(`Moving: Linear ${twist.linear.x.toFixed(2)}, Angular ${twist.angular.z.toFixed(2)}`);
    }, 100);
}

function stopMove() {
    if (moveInterval) {
        clearInterval(moveInterval);
        moveInterval = null;
    }
    sendStopCommand();
}

function sendStopCommand() {
    const stopTwist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
    });

    if (!cmdVel) {
        cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/Twist'
        });
    }
    cmdVel.publish(stopTwist);
    console.log("Stop command sent.");
}

// --- 확대/축소 (Zoom) 관련 함수 ---
function setZoom(level) {
    currentZoomLevel = Math.max(minZoomLevel, Math.min(maxZoomLevel, level));
    adjustPanOffset(); // 줌 레벨 변경 시 패닝 제한 재조정
    drawMapAndRobot();
    console.log("Zoom level set to:", currentZoomLevel);
}

function zoomIn() {
    setZoom(currentZoomLevel + zoomStep);
}

function zoomOut() {
    setZoom(currentZoomLevel - zoomStep);
}

// --- 패닝(Panning) 함수들 (수정됨) ---
function startDragging(e) {
    if (e.button === 0) {
        isDragging = true;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;
        canvasContainer.style.cursor = 'grabbing';
    }
}

function dragMap(e) {
    if (!isDragging || !mapInfo) return; // mapInfo가 없으면 계산 불가

    const dx = e.clientX - lastMouseX;
    const dy = e.clientY - lastMouseY;

    // 현재 캔버스에 표시되는 맵의 픽셀 크기
    const scaledMapWidth = mapInfo.width * currentZoomLevel;
    const scaledMapHeight = mapInfo.height * currentZoomLevel;

    // 캔버스 자체의 보이는 영역 크기 (CSS로 설정된 캔버스 실제 너비/높이)
    const canvasVisibleWidth = mapCanvas.offsetWidth;
    const canvasVisibleHeight = mapCanvas.offsetHeight;

    // 새로운 offsetX, offsetY를 계산
    let newOffsetX = offsetX + dx;
    let newOffsetY = offsetY + dy;

    // X축 제한
    // 맵이 캔버스보다 작을 때: 맵을 중앙에 고정
    if (scaledMapWidth < canvasVisibleWidth) {
        newOffsetX = (canvasVisibleWidth - scaledMapWidth) / 2;
    } else {
        // 맵이 캔버스보다 클 때: 맵의 좌측 끝이 캔버스의 좌측 끝을 넘지 않도록 (0 이상)
        // 맵의 우측 끝이 캔버스의 우측 끝을 넘지 않도록 (캔버스 너비 - 스케일된 맵 너비) 이하
        newOffsetX = Math.min(newOffsetX, 0); // 맵의 좌측 끝이 캔버스의 좌측 끝보다 오른쪽으로 가지 않도록 (최대 0)
        newOffsetX = Math.max(newOffsetX, canvasVisibleWidth - scaledMapWidth); // 맵의 우측 끝이 캔버스의 우측 끝보다 왼쪽으로 가지 않도록 (최소 [캔버스 너비 - 스케일된 맵 너비])
    }

    // Y축 제한
    // 맵이 캔버스보다 작을 때: 맵을 중앙에 고정
    if (scaledMapHeight < canvasVisibleHeight) {
        newOffsetY = (canvasVisibleHeight - scaledMapHeight) / 2;
    } else {
        // 맵이 캔버스보다 클 때: 맵의 상단 끝이 캔버스의 상단 끝을 넘지 않도록 (0 이상)
        // 맵의 하단 끝이 캔버스의 하단 끝을 넘지 않도록 (캔버스 높이 - 스케일된 맵 높이) 이하
        newOffsetY = Math.min(newOffsetY, 0); // 맵의 상단 끝이 캔버스의 상단 끝보다 아래쪽으로 가지 않도록 (최대 0)
        newOffsetY = Math.max(newOffsetY, canvasVisibleHeight - scaledMapHeight); // 맵의 하단 끝이 캔버스의 하단 끝보다 위쪽으로 가지 않도록 (최소 [캔버스 높이 - 스케일된 맵 높이])
    }

    offsetX = newOffsetX;
    offsetY = newOffsetY;

    lastMouseX = e.clientX;
    lastMouseY = e.clientY;

    drawMapAndRobot();
}

function stopDragging() {
    isDragging = false;
    canvasContainer.style.cursor = 'grab';
}

// 맵이 로드되거나 줌 레벨이 변경될 때 패닝 오프셋을 재조정하는 함수 (새로 추가)
function adjustPanOffset() {
    if (!mapInfo) return;

    const scaledMapWidth = mapInfo.width * currentZoomLevel;
    const scaledMapHeight = mapInfo.height * currentZoomLevel;

    const canvasVisibleWidth = mapCanvas.offsetWidth;
    const canvasVisibleHeight = mapCanvas.offsetHeight;

    let newOffsetX = offsetX;
    let newOffsetY = offsetY;

    // X축 제한
    if (scaledMapWidth < canvasVisibleWidth) {
        newOffsetX = (canvasVisibleWidth - scaledMapWidth) / 2;
    } else {
        newOffsetX = Math.min(newOffsetX, 0);
        newOffsetX = Math.max(newOffsetX, canvasVisibleWidth - scaledMapWidth);
    }

    // Y축 제한
    if (scaledMapHeight < canvasVisibleHeight) {
        newOffsetY = (canvasVisibleHeight - scaledMapHeight) / 2;
    } else {
        newOffsetY = Math.min(newOffsetY, 0);
        newOffsetY = Math.max(newOffsetY, canvasVisibleHeight - scaledMapHeight);
    }

    // 오프셋이 실제로 변경되었을 때만 다시 그립니다.
    if (offsetX !== newOffsetX || offsetY !== newOffsetY) {
        offsetX = newOffsetX;
        offsetY = newOffsetY;
        drawMapAndRobot();
    }
}


// --- DOM 콘텐츠가 모두 로드된 후 실행될 함수 ---
document.addEventListener('DOMContentLoaded', function() {
    mapCanvas = document.getElementById('mapCanvas');
    locationDisplay = document.getElementById('location_display');
    canvasContainer = document.querySelector('.canvas-container');

    if (mapCanvas) mapCtx = mapCanvas.getContext('2d');

    if (!mapCanvas || !mapCtx || !locationDisplay || !canvasContainer) {
        console.error("ERROR: 필수 DOM 요소가 HTML에 없습니다! ID를 확인하세요.");
        return;
    }

    // 초기 로드 시 캔버스 크기 조정 및 패닝 오프셋 조정
    // 이 부분은 맵 데이터가 로드된 후 subscribeToMap에서 처리하므로 주석 처리하거나 제거해도 됩니다.
    // 하지만, mapCanvas.offsetWidth/Height가 초기 CSS 값으로 설정되어야 정확합니다.
    // adjustPanOffset(); // 초기 로드 시 패닝 제한 재조정

    if (ros.isConnected) {
        subscribeToOdom();
        subscribeToMap();
        subscribeToTF();
    }

    canvasContainer.addEventListener('mousedown', startDragging);
    canvasContainer.addEventListener('mousemove', dragMap);
    canvasContainer.addEventListener('mouseup', stopDragging);
    canvasContainer.addEventListener('mouseleave', stopDragging);

    // 윈도우 크기 변경 시에도 패닝 오프셋을 재조정하여 맵이 중앙에 유지되도록 합니다.
    window.addEventListener('resize', adjustPanOffset);
});