// Browser client: ROS remains on the server; this page uses HTTP and WebSocket.
const $ = (id) => document.getElementById(id);
const tabButtons = document.querySelectorAll('.tab-btn');
const tabPanels = document.querySelectorAll('.tab-panel');
tabButtons.forEach((btn) => btn.addEventListener('click', () => {
  tabButtons.forEach((b) => b.classList.toggle('active', b === btn));
  tabPanels.forEach((p) => p.classList.toggle('active', p.id === `tab-${btn.dataset.tab}`));
}));

let socket, joyTimer, move = {v: 0, w: 0};
window.robotPlan = [];
function request(path, body) {
  return fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)})
    .then(async (r) => { const data = await r.json().catch(() => ({})); if (!r.ok) throw Error(data.detail || r.statusText); return data; });
}
function led(id, color) { const el = $(id); el.classList.remove('green', 'amber', 'red'); if (color) el.classList.add(color); }
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
  if (s.estop != null) { led('safety-estop', s.estop ? 'red' : null); $('estop').textContent = s.estop ? 'RESET E-STOP' : 'E-STOP'; }
  if (data.health) led('system-led', 'green');
  if (data.mission_status?.active) {
    const m = data.mission_status.active;
    $('mission-id').textContent = m.id || '--'; $('mission-from').textContent = m.pickup || '--'; $('mission-to').textContent = m.dropoff || '--';
    led('mission-progress', 'amber');
  }
}
function connect() {
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  socket.onopen = () => { $('system-text').textContent = 'System connected'; led('system-led', 'green'); };
  socket.onclose = () => { $('system-text').textContent = 'Connection lost'; led('system-led', 'red'); stopJoy(); setTimeout(connect, 1500); };
  socket.onerror = () => socket.close();
  socket.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'snapshot') render(msg.data);
    else if (['status', 'pose', 'battery', 'mission_status', 'health'].includes(msg.type)) render({[msg.type]: msg.data, status: msg.type === 'status' ? msg.data : undefined});
    else if (msg.type === 'map_updated') refreshMap();
    else if (msg.type === 'plan') { window.robotPlan = msg.data.points || []; refreshMap(); }
  };
}
connect();

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
function startJoy(cmd) { move = direction(cmd); if (joyTimer) return; const send = () => { if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({type: 'joy', ...move})); }; send(); joyTimer = setInterval(send, 100); }
function stopJoy() { move = {v: 0, w: 0}; if (joyTimer) { clearInterval(joyTimer); joyTimer = null; } if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({type: 'joy', v: 0, w: 0})); }
document.querySelectorAll('[data-cmd]').forEach((button) => {
  const cmd = button.dataset.cmd;
  if (['forward', 'backward', 'left', 'right'].includes(cmd)) {
    button.addEventListener('pointerdown', (e) => { e.preventDefault(); startJoy(cmd); });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach((ev) => button.addEventListener(ev, stopJoy));
  } else if (cmd === 'lift-up' || cmd === 'lift-down') {
    button.addEventListener('pointerdown', () => request('/api/lift', {cmd: cmd === 'lift-up' ? 1 : 2}).catch((e) => alert(e.message)));
    ['pointerup', 'pointercancel', 'pointerleave'].forEach((ev) => button.addEventListener(ev, () => request('/api/lift', {cmd: 0}).catch(() => {})));
  } else if (cmd.startsWith('lights-')) button.addEventListener('click', () => request('/api/lights', {relays: Array(4).fill(cmd === 'lights-on' ? 1 : 0)}).catch((e) => alert(e.message)));
});
window.addEventListener('keydown', (e) => { if (e.repeat) return; const d = {w: 'forward', s: 'backward', a: 'left', d: 'right'}[e.key.toLowerCase()]; if (d) startJoy(d); });
window.addEventListener('keyup', (e) => { if ('wasd'.includes(e.key.toLowerCase())) stopJoy(); });
window.addEventListener('blur', stopJoy);
