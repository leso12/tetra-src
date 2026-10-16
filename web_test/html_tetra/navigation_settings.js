/* Safe staged-navigation settings client. Does not publish cmd_vel or restart nodes. */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const fields = Array.from(document.querySelectorAll('[data-setting]'));
  let ros = null, getService = null, saveService = null, revision = null;
  let ready = false, laserSeen = 0, odomSeen = 0, topics = [];

  $('wsUrl').value = 'ws://' + (window.location.hostname || '192.168.33.2') + ':9090';

  function notice(message, level) {
    const el = $('message');
    el.textContent = message;
    el.className = 'notice' + (level ? ' ' + level : '');
  }
  function badge(message, kind) {
    $('connectionBadge').textContent = message;
    $('connectionBadge').className = 'badge ' + kind;
  }
  function enabled(value) {
    ready = value;
    fields.forEach(el => { el.disabled = !value; });
    $('saveButton').disabled = !value;
    $('reloadButton').disabled = !value;
  }
  function disconnect() {
    enabled(false);
    revision = null;
    topics.forEach(t => { try { t.unsubscribe(); } catch (_) {} });
    topics = [];
    if (ros) { try { ros.close(); } catch (_) {} }
    ros = null;
    getService = null;
    saveService = null;
    laserSeen = 0;
    odomSeen = 0;
  }
  function serviceName(action) {
    return '/' + $('namespace').value.trim().replace(/^\/+|\/+$/g, '') + '/nav_tuning/' + action;
  }
  function connect() {
    disconnect();
    const url = $('wsUrl').value.trim();
    const ns = $('namespace').value.trim();
    if (!/^wss?:\/\/[^ ]+:\d{2,5}\/?$/.test(url) || !/^[a-zA-Z][a-zA-Z0-9_]*$/.test(ns)) {
      notice('WebSocket 주소와 ROS 네임스페이스를 확인해 주세요.', 'error');
      return;
    }
    if (typeof ROSLIB === 'undefined') { notice('ROSLIB 스크립트를 불러오지 못했습니다.', 'error'); return; }
    notice('ROSBridge에 연결 중…');
    badge('연결 중', 'neutral');
    const connection = new ROSLIB.Ros({url: url});
    ros = connection;
    connection.on('connection', () => {
      if (ros !== connection) return;
      badge('ROS 연결됨', 'ok');
      getService = new ROSLIB.Service({ros: connection,name: serviceName('get'),serviceType:'tetraDS_2dnav/GetNavigationTuning'});
      saveService = new ROSLIB.Service({ros: connection,name: serviceName('save'),serviceType:'tetraDS_2dnav/SetNavigationTuning'});
      subscribe(connection, '/' + ns + '/scan', 'sensor_msgs/LaserScan', () => {laserSeen = Date.now();});
      subscribe(connection, '/' + ns + '/odometry/filtered', 'nav_msgs/Odometry', () => {odomSeen = Date.now();});
      reload();
    });
    connection.on('close', () => {
      if (ros !== connection) return;
      badge('연결 끊김','bad');
      enabled(false);
      notice('연결이 끊어졌습니다. 저장할 수 없습니다. 재연결 후 설정을 다시 읽으세요.','error');
    });
    connection.on('error', () => {
      if (ros !== connection) return;
      badge('연결 오류','bad');
      enabled(false);
      notice('WebSocket 연결 실패. 주소, 네트워크, ROSBridge 실행 상태를 확인하세요.','error');
    });
  }
  function subscribe(connection, name, messageType, callback) {
    const topic = new ROSLIB.Topic({ros:connection,name,messageType,throttle_rate:400});
    topic.subscribe(callback);
    topics.push(topic);
  }
  function loadResponse(response) {
    if (!response || !response.success) {
      enabled(false);
      notice('설정 읽기 실패: ' + (response && response.message || '설정 서비스가 실행 중인지 확인하세요.'),'error');
      return;
    }
    let payload;
    try { payload = JSON.parse(response.settings_json); }
    catch (_) { notice('설정 응답을 읽지 못했습니다.','error'); enabled(false); return; }
    revision = payload.revision;
    fields.forEach(el => {
      const name = el.dataset.setting;
      el.value = Number(payload.values[name]);
      const bounds = payload.bounds[name];
      if (bounds) {el.min = bounds[0]; el.max = bounds[1];}
      const active = payload.active[name];
      document.querySelector('[data-active="' + name + '"]').textContent =
        '현재 적용: ' + (Number.isFinite(active) ? active.toFixed(2) + ' m' : '확인 불가');
    });
    $('applyState').textContent = !payload.navigation_running ? '내비게이션 미실행' :
      payload.pending_restart ? '재시작 필요' : '설정 일치';
    $('applyDetail').textContent = payload.pending_restart ?
      '저장 후 안전 정지하여 내비게이션 재시작 필요' : '저장값과 ROS 파라미터 일치';
    enabled(true);
    notice('설정 로드 완료. 화면에 표시된 값은 저장값이며, 실제 적용값은 각 항목 아래에서 확인하세요.','success');
  }
  function reload() {
    enabled(false);
    if (!getService) return;
    getService.callService(new ROSLIB.ServiceRequest({}), loadResponse, () => {
      enabled(false);
      notice('ROS 설정 서비스 호출 실패. catkin 빌드 및 nav_tuning_server 실행 상태를 확인하세요.','error');
    });
  }
  function save(event) {
    event.preventDefault();
    if (!ready || !ros || !ros.isConnected || !revision) return;
    const values = {};
    for (const el of fields) {
      const num = Number(el.value);
      if (!el.value.trim() || !Number.isFinite(num) || num < Number(el.min) || num > Number(el.max)) {
        notice('입력 범위 오류: ' + el.closest('.setting').querySelector('b').textContent,'error');
        el.focus(); return;
      }
      values[el.dataset.setting] = num;
    }
    if (values.lidar_clear < values.lidar_mark || values.teb_inflation < values.teb_min_obstacle) {
      notice('LiDAR Clearing ≥ 장애물 반영 거리, TEB Inflation ≥ 최소 이격거리를 지켜주세요.','error'); return;
    }
    if (!window.confirm('설정 파일에 변경 사항을 저장할까요?\n주행 중 즉시 적용되지 않으며 안전 정지 후 내비게이션 재시작이 필요합니다.')) return;
    enabled(false);
    saveService.callService(new ROSLIB.ServiceRequest({
      settings_json: JSON.stringify({revision,values})
    }), (result) => {
      if (!result || !result.success) {
        notice('저장 거부: ' + (result && result.message || '알 수 없는 오류') + ' · 변경 전 값은 유지됩니다.','error');
        reload(); return;
      }
      loadResponse(result);
      notice('설정 파일 저장 완료. 내비게이션이 안전하게 정지한 뒤 재시작해야 적용됩니다.','success');
    }, () => {
      notice('저장 결과를 확인할 수 없습니다. 다시 연결해 실제 파일 상태를 읽으세요.','error');
      enabled(false);
    });
  }
  function sensorStatus(last, state, detail) {
    const age = last ? Date.now() - last : Infinity;
    $(state).textContent = age < 3000 ? '수신 정상' : '수신 없음';
    $(detail).textContent = age < 3000 ? '최근 ' + (age / 1000).toFixed(1) + '초 전 수신' : '센서 토픽 또는 연결 상태 확인';
  }
  $('connectButton').addEventListener('click',connect);
  $('reloadButton').addEventListener('click',reload);
  $('tuningForm').addEventListener('submit',save);
  setInterval(() => {sensorStatus(laserSeen,'laserState','laserDetail');sensorStatus(odomSeen,'odomState','odomDetail');},1000);
})();
