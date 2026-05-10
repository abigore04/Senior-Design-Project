(function () {
  'use strict';

  var els = {};
  var socket = null;
  var connected = false;
  var lastMetalTs = 0;
  var metalResetTimer = null;
  var lastKnownMarkerCount = 0;
  var lastNewMarkerTime = 0;
  var cfgKey = 'sdp_nizami_manual_monitor_config_v1';

  function $(id) { return document.getElementById(id); }

  function nowTime() {
    var d = new Date();
    return d.toTimeString().split(' ')[0];
  }

  function log(msg) {
    var div = document.createElement('div');
    div.className = 'logLine';
    div.innerHTML = '<span class="logTime">' + nowTime() + '</span>' + escapeHtml(msg);
    els.logBox.insertBefore(div, els.logBox.firstChild);
    while (els.logBox.children.length > 80) {
      els.logBox.removeChild(els.logBox.lastChild);
    }
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, function (c) {
      return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];
    });
  }

  function setBadge(el, text, cls) {
    el.textContent = text;
    el.className = 'badge ' + cls;
  }

  function getCfg() {
    return {
      host: els.hostInput.value.trim() || '127.0.0.1',
      rosPort: els.rosPortInput.value.trim() || '9090',
      videoPort: els.videoPortInput.value.trim() || '8080',
      cameraTopic: els.cameraTopicInput.value.trim() || '/usb_cam/image_raw',
      metalTopic: els.metalTopicInput.value.trim() || '/metal_detections',
      odomTopic: els.odomTopicInput.value.trim() || '/odom',
      detectorTopic: els.detectorTopicInput.value.trim() || '/metal_detector_status'
    };
  }

  function loadCfg() {
    try {
      var saved = JSON.parse(localStorage.getItem(cfgKey) || '{}');
      if (saved.host) els.hostInput.value = saved.host;
      if (saved.rosPort) els.rosPortInput.value = saved.rosPort;
      if (saved.videoPort) els.videoPortInput.value = saved.videoPort;
      if (saved.cameraTopic) els.cameraTopicInput.value = saved.cameraTopic;
      if (saved.metalTopic) els.metalTopicInput.value = saved.metalTopic;
      if (saved.odomTopic) els.odomTopicInput.value = saved.odomTopic;
      if (saved.detectorTopic) els.detectorTopicInput.value = saved.detectorTopic;
    } catch (e) {}
  }

  function saveCfg() {
    localStorage.setItem(cfgKey, JSON.stringify(getCfg()));
    log('Configuration saved.');
  }

  function buildVideoUrl() {
    var c = getCfg();
    var topic = c.cameraTopic;
    return 'http://' + c.host + ':' + c.videoPort + '/stream?topic=' + topic + '&type=mjpeg&quality=80&client_id=phone-monitor';
  }

  function reloadVideo() {
    var c = getCfg();
    var url = buildVideoUrl() + '&t=' + Date.now();
    els.cameraTopicLabel.textContent = c.cameraTopic;
    els.videoUrlText.textContent = url;
    els.cameraHint.style.display = 'none';
    els.cameraStream.src = url;
    log('Video reloaded: ' + c.cameraTopic);
  }

  function wsSend(obj) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify(obj));
  }

  function subscribe(topic, type) {
    wsSend({ op: 'subscribe', topic: topic, type: type || undefined, throttle_rate: 100 });
    log('Subscribed to ' + topic);
  }

  function unsubscribe(topic) {
    wsSend({ op: 'unsubscribe', topic: topic });
  }

  function connect() {
    disconnect(false);
    var c = getCfg();
    var url = 'ws://' + c.host + ':' + c.rosPort;
    log('Connecting to ' + url);
    setBadge(els.rosBadge, 'ROS connecting…', 'warn');

    try {
      socket = new WebSocket(url);
    } catch (e) {
      log('WebSocket create error: ' + e.message);
      setBadge(els.rosBadge, 'ROS error', 'bad');
      return;
    }

    socket.onopen = function () {
      connected = true;
      setBadge(els.rosBadge, 'ROS connected', 'good');
      log('ROSBridge connected.');
      subscribe(c.odomTopic, 'nav_msgs/Odometry');
      subscribe(c.metalTopic);
      subscribe(c.detectorTopic, 'std_msgs/String');
      reloadVideo();
    };

    socket.onmessage = function (event) {
      var data;
      try { data = JSON.parse(event.data); } catch (e) { return; }
      if (data.op !== 'publish') return;
      handlePublish(data.topic, data.msg);
    };

    socket.onerror = function () {
      log('WebSocket error. Check IP, port, and network path.');
      setBadge(els.rosBadge, 'ROS error', 'bad');
    };

    socket.onclose = function () {
      connected = false;
      setBadge(els.rosBadge, 'ROS disconnected', 'bad');
      log('ROSBridge disconnected.');
    };
  }

  function disconnect(doLog) {
    if (socket) {
      try {
        var c = getCfg();
        unsubscribe(c.odomTopic);
        unsubscribe(c.metalTopic);
        unsubscribe(c.detectorTopic);
        socket.close();
      } catch (e) {}
    }
    socket = null;
    connected = false;
    if (doLog !== false) log('Disconnected.');
  }

  function handlePublish(topic, msg) {
    var c = getCfg();
    if (topic === c.odomTopic) return handleOdom(msg);
    if (topic === c.metalTopic) return handleMetal(msg);
    if (topic === c.detectorTopic) return handleDetectorStatus(msg);
  }

  function handleOdom(msg) {
    try {
      var lin = Number(msg.twist.twist.linear.x || 0);
      var ang = Number(msg.twist.twist.angular.z || 0);
      els.linearSpeed.textContent = lin.toFixed(2);
      els.angularSpeed.textContent = ang.toFixed(2);
      els.odomStatus.textContent = 'Receiving /odom';
    } catch (e) {
      els.odomStatus.textContent = 'Odom parse error';
    }
  }

  function handleDetectorStatus(msg) {
    var raw = extractString(msg).toLowerCase();
    var connected = raw.indexOf('connected') >= 0 && raw.indexOf('unconnected') < 0;
    if (connected) {
      setBadge(els.detectorBadge, 'metal_detector connected', 'good');
      els.detectorState.textContent = 'connected';
    } else {
      setBadge(els.detectorBadge, 'metal_detector unconnected', 'bad');
      els.detectorState.textContent = 'unconnected';
    }
  }

  function handleMetal(msg) {
    var detected = false;
    var raw = '';
    if (msg && msg.markers && msg.markers.length > lastKnownMarkerCount) {
      lastKnownMarkerCount = msg.markers.length;
      lastNewMarkerTime = Date.now();
      detected = true;
      raw = 'METAL_' + msg.markers.length + '_markers';
    } else if (msg && msg.markers && Date.now() - lastNewMarkerTime < 4000 && lastNewMarkerTime > 0) {
      detected = true;
      raw = 'METAL_' + msg.markers.length + '_markers';
    } else {
      var parsed = parseMetal(msg);
      detected = parsed.detected;
      raw = parsed.raw;
    }
    els.metalConfidence.textContent = msg && msg.markers ? msg.markers.length : '--';
    els.metalRaw.textContent = raw;

    if (detected) {
      lastMetalTs = Date.now();
      els.metalState.textContent = 'METAL DETECTED';
      els.metalState.className = 'metalState detected';
      els.lastMetalTime.textContent = nowTime();
      setBadge(els.metalBadge, 'METAL DETECTED', 'bad');
      log('Metal detection: ' + parsed.raw);

      if (metalResetTimer) clearTimeout(metalResetTimer);
      metalResetTimer = setTimeout(function () {
        els.metalState.textContent = 'NO METAL';
        els.metalState.className = 'metalState idle';
        setBadge(els.metalBadge, 'No metal', 'neutral');
      }, 3000);
    }
  }

  function extractString(msg) {
    if (msg == null) return '';
    if (typeof msg === 'string') return msg;
    if (typeof msg.data === 'string') return msg.data;
    return JSON.stringify(msg);
  }

  function parseMetal(msg) {
    var raw = extractString(msg);
    var lower = raw.toLowerCase();
    var detected = false;
    var confidence = '—';

    if (typeof msg.detected === 'boolean') detected = msg.detected;
    if (typeof msg.is_metal === 'boolean') detected = msg.is_metal;
    if (typeof msg.metal === 'boolean') detected = msg.metal;

    if (!detected) {
      if (lower.indexOf('detected') >= 0 || lower.indexOf('metal') >= 0 || lower.indexOf('hit') >= 0) {
        if (lower.indexOf('no metal') < 0 && lower.indexOf('false') < 0) detected = true;
      }
      if (raw === '1' || lower === 'true') detected = true;
    }

    if (typeof msg.confidence !== 'undefined') confidence = String(msg.confidence);
    else if (typeof msg.strength !== 'undefined') confidence = String(msg.strength);
    else if (typeof msg.value !== 'undefined') confidence = String(msg.value);
    else {
      var m = raw.match(/(?:conf|confidence|strength|value)\s*[:=]\s*([0-9.]+)/i);
      if (m) confidence = m[1];
    }

    return { detected: detected, confidence: confidence, raw: raw };
  }

  function init() {
    ['hostInput','rosPortInput','videoPortInput','cameraTopicInput','metalTopicInput','odomTopicInput','detectorTopicInput','saveBtn','connectBtn','disconnectBtn','reloadVideoBtn','clearLogsBtn','rosBadge','metalBadge','detectorBadge','linearSpeed','angularSpeed','odomStatus','metalState','metalConfidence','lastMetalTime','detectorState','metalRaw','cameraStream','cameraHint','videoUrlText','cameraTopicLabel','logBox'].forEach(function (id) {
      els[id] = $(id);
    });

    loadCfg();
    els.saveBtn.onclick = saveCfg;
    els.connectBtn.onclick = connect;
    els.disconnectBtn.onclick = function () { disconnect(true); };
    els.reloadVideoBtn.onclick = reloadVideo;
    els.clearLogsBtn.onclick = function () { els.logBox.innerHTML = ''; };
    els.cameraStream.onerror = function () {
      els.cameraHint.style.display = 'block';
      els.cameraHint.textContent = 'Camera stream not loaded. Check web_video_server, topic name, and port 8080.';
    };
    els.cameraStream.onload = function () {
      els.cameraHint.style.display = 'none';
    };
    log('Dashboard ready. Landscape monitor only.');
  }

  window.addEventListener('load', init);
})();
