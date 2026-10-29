// Browser client: ROS remains on the server; this page uses HTTP and WebSocket.
const $ = (id) => document.getElementById(id);
const tabButtons = document.querySelectorAll('.tab-btn');
const tabPanels = document.querySelectorAll('.tab-panel');
tabButtons.forEach((btn) => btn.addEventListener('click', () => {
  tabButtons.forEach((b) => b.classList.toggle('active', b === btn));
  tabPanels.forEach((p) => p.classList.toggle('active', p.id === `tab-${btn.dataset.tab}`));
}));

let socket, joyTimer, move = {v: 0, w: 0};
let online = null, retry = 0, lastMessage = 0, reconnectTimer = null;
const STALE_MS = 3000, MAX_RETRY_MS = 5000;
window.robotPlan = [];
const displayIds = [
  'status-state', 'status-position', 'status-heading', 'status-speed',
  'status-battery', 'status-fork', 'status-package', 'status-quantity',
  'status-weight', 'status-eta', 'mission-id', 'mission-from', 'mission-to',
];
const ledIds = ['mission-progress', 'mission-done', 'safety-normal', 'safety-obstacle', 'safety-estop'];
const placeholders = Object.fromEntries(displayIds.map((id) => [id, $(id).textContent]));
function request(path, body) {
  return fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)})
    .then(async (r) => { const data = await r.json().catch(() => ({})); if (!r.ok) throw Error(data.detail || r.statusText); return data; });
}
function led(id, color) { const el = $(id); el.classList.remove('green', 'amber', 'red'); if (color) el.classList.add(color); }
function setOnline(value) {
  if (online === value) return;
  online = value;
  document.querySelectorAll('.ctrl-btn, .mode-btn').forEach((button) => {
    button.disabled = !online;
    button.style.opacity = online ? '' : '0.5';
  });
  if (online) {
    $('system-text').textContent = 'System online'; led('system-led', 'green');
    return;
  }
  stopJoy(); stopLift();
  displayIds.forEach((id) => { $(id).textContent = placeholders[id]; });
  ledIds.forEach((id) => led(id, null));
  $('battery-bar').style.width = '0%';
  $('system-text').textContent = 'Offline - reconnecting...'; led('system-led', 'red');
}
function setMode(mode) {
  const manual = mode === 'manual';
  $('mode-auto').classList.toggle('active', !manual); $('mode-manual').classList.toggle('active', manual);
  $('control-panel').classList.toggle('hidden', !manual);
}
function render(data = {}) {
  const s = data.status || data;
  if (s.state) $('status-state').textContent = s.state;
  const pose = s.pose || data.pose;
  if (pose) { window.robotPose = pose; $('status-position').textContent = `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)} m`; $('status-heading').textContent = `${pose.heading_deg.toFixed(1)}°`; }
  if (s.speed != null) $('status-speed').textContent = `${Number(s.speed).toFixed(2)} m/s`;
  const battery = typeof s.battery === 'object' ? s.battery.percentage : s.battery;
  if (battery != null) { $('status-battery').textContent = `${Math.round(battery)}%`; $('battery-bar').style.width = `${battery}%`; }
  if (s.fork_height_mm != null) $('status-fork').textContent = `${(s.fork_height_mm / 10).toFixed(1)} cm`;
  if (s.mode) setMode(s.mode.toLowerCase());
  if (s.estop != null) { led('safety-estop', s.estop ? 'red' : null); led('safety-normal', s.estop ? null : 'green'); $('estop').textContent = s.estop ? 'RESET E-STOP' : 'E-STOP'; }
  if (data.health) led('system-led', 'green');
  if (data.mission_status?.active) {
    const m = data.mission_status.active;
    $('mission-id').textContent = m.id || '--'; $('mission-from').textContent = m.pickup || '--'; $('mission-to').textContent = m.dropoff || '--';
    led('mission-progress', 'amber');
  }
}
function connect() {
  if (socket && socket.readyState !== WebSocket.CLOSED) return;
  $('system-text').textContent = 'Connecting...';
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  socket.onopen = () => {};
  socket.onclose = () => {
    setOnline(false);
    const delay = Math.min(1000 * (2 ** retry), MAX_RETRY_MS);
    retry += 1;
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connect, delay);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = (e) => {
    let msg;
    try { msg = JSON.parse(e.data); } catch (_) { return; }
    lastMessage = Date.now(); retry = 0; setOnline(true);
    if (msg.type === 'snapshot') render(msg.data);
    else if (['status', 'pose', 'battery', 'mission_status', 'health'].includes(msg.type)) render({[msg.type]: msg.data, status: msg.type === 'status' ? msg.data : undefined});
    else if (msg.type === 'map_updated') refreshMap();
    else if (msg.type === 'plan') { window.robotPlan = msg.data.points || []; refreshMap(); }
  };
}
connect();

// A connected socket can still go quiet (for example if the server worker stalls).
setInterval(() => {
  if (online && lastMessage && Date.now() - lastMessage > STALE_MS) {
    setOnline(false);
    if (socket?.readyState === WebSocket.OPEN) socket.close();
  }
}, 1000);

async function refreshMap() {
  try {
    const [mr, ir] = await Promise.all([fetch('/api/map/meta'), fetch(`/api/map.png?t=${Date.now()}`)]);
    if (!mr.ok || !ir.ok) return;
    const meta = await mr.json(), bitmap = await createImageBitmap(await ir.blob());
    const canvas = $('map-canvas'), ctx = canvas.getContext('2d'); canvas.width = bitmap.width; canvas.height = bitmap.height; ctx.drawImage(bitmap, 0, 0);
    const p = window.robotPose;
    if (window.robotPlan.length) {
      ctx.strokeStyle = '#e4a11b'; ctx.lineWidth = 2; ctx.beginPath();
      window.robotPlan.forEach((pt, i) => { const x = (pt.x - meta.origin.x) / meta.resolution, y = canvas.height - (pt.y - meta.origin.y) / meta.resolution; if (!i) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
      ctx.stroke();
    }
    if (p) { const x = (p.x - meta.origin.x) / meta.resolution, y = canvas.height - (p.y - meta.origin.y) / meta.resolution; ctx.fillStyle = '#1caa72'; ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill(); }
  } catch (_) { /* map is unavailable until SLAM publishes it */ }
}
setInterval(refreshMap, 2000);
$('camera-feed').src = '/stream/camera';

async function changeMode(mode) {
  const previous = $('mode-manual').classList.contains('active') ? 'manual' : 'auto';
  // Update the control immediately; the backend response and ROS echo follow.
  setMode(mode.toLowerCase());
  try { await request('/api/mode', {mode}); }
  catch (e) { setMode(previous); alert(e.message); }
}
$('mode-auto').addEventListener('click', () => changeMode('AUTO'));
$('mode-manual').addEventListener('click', () => changeMode('MANUAL'));
$('estop').addEventListener('click', () => request('/api/estop', {active: $('estop').textContent !== 'RESET E-STOP'}).catch((e) => alert(e.message)));

function direction(cmd) { return ({forward: {v: .3, w: 0}, backward: {v: -.3, w: 0}, left: {v: 0, w: .5}, right: {v: 0, w: -.5}}[cmd] || {v: 0, w: 0}); }
function startJoy(cmd) { if (!online || joyTimer) return; move = direction(cmd); const send = () => { if (socket?.readyState === WebSocket.OPEN && online) socket.send(JSON.stringify({type: 'joy', ...move})); }; send(); joyTimer = setInterval(send, 100); }
function stopJoy() { move = {v: 0, w: 0}; if (joyTimer) { clearInterval(joyTimer); joyTimer = null; } if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({type: 'joy', v: 0, w: 0})); }
let liftTimer = null, liftCommand = null;
function startLift(cmd) {
  if (!online || liftTimer) return;
  liftCommand = cmd;
  const send = () => request('/api/lift', {cmd: liftCommand}).catch(() => stopLift());
  send(); liftTimer = setInterval(send, 100);
}
function stopLift() {
  if (liftTimer) { clearInterval(liftTimer); liftTimer = null; }
  if (liftCommand !== null) request('/api/lift', {cmd: 0}).catch(() => {});
  liftCommand = null;
}
document.querySelectorAll('[data-cmd]').forEach((button) => {
  const cmd = button.dataset.cmd;
  if (['forward', 'backward', 'left', 'right'].includes(cmd)) {
    button.style.touchAction = 'none';
    button.addEventListener('contextmenu', (e) => e.preventDefault());
    button.addEventListener('pointerdown', (e) => { e.preventDefault(); button.setPointerCapture(e.pointerId); startJoy(cmd); });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) => button.addEventListener(ev, stopJoy));
  } else if (cmd === 'lift-up' || cmd === 'lift-down') {
    button.style.touchAction = 'none';
    button.addEventListener('contextmenu', (e) => e.preventDefault());
    button.addEventListener('pointerdown', (e) => { e.preventDefault(); button.setPointerCapture(e.pointerId); startLift(cmd === 'lift-up' ? 1 : 2); });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) => button.addEventListener(ev, stopLift));
  } else if (cmd.startsWith('lights-')) button.addEventListener('click', () => request('/api/lights', {relays: Array(4).fill(cmd === 'lights-on' ? 1 : 0)}).catch((e) => alert(e.message)));
});
window.addEventListener('keydown', (e) => { if (e.repeat) return; const d = {w: 'forward', s: 'backward', a: 'left', d: 'right'}[e.key.toLowerCase()]; if (d) startJoy(d); });
window.addEventListener('keyup', (e) => { if ('wasd'.includes(e.key.toLowerCase())) stopJoy(); });
function stopAllHolds() { stopJoy(); stopLift(); }
window.addEventListener('blur', stopAllHolds);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopAllHolds(); });
setOnline(false);
