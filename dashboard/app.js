const $ = (id) => document.getElementById(id);
const format = (n, digits = 2) => Number.isFinite(n) ? n.toFixed(digits).replace('.', ',') : '—';
const demo = new URLSearchParams(location.search).get('preview') === '1';
const messages = {
  controller_busy: 'Robot đang được điều khiển trên thiết bị khác.',
  imu_not_ready: 'Hiệu chỉnh IMU để dùng phần Đặt lệnh.',
  not_ready: 'Chờ robot dừng hoặc hoàn tất hiệu chỉnh để đặt hành trình.',
  command_rejected: 'Kiểm tra quãng đường, góc quay hoặc tăng thời gian đặt.',
  cannot_clear: 'Để xe đứng yên rồi xóa lỗi.',
  motor_fault: 'Robot có lỗi motor. Kiểm tra xe trước khi tiếp tục.'
};
let state = null, online = false, owned = false, wanted = false, ws = null;
let seq = 0, lastFrame = 0, lastMs = null, boot = null;
let pending = false, message = '', messageError = false;
let retry = null;
let statusTimer = null, statusRequest = null, statusEpoch = 0, pageLeaving = false;
const waiting = new Map(), pointers = new Map(), keysDown = new Set();
let previewState = {
  boot: 'PREVIEW', ms: 0, state: 'IDLE', fault: 0, ready: true, remote_ready: true,
  stationary: true, owned: false, manual: false, run: 0, turning: false,
  speed: 0, rpm: [0, 0], sp: [0, 0], distance: 0, target: 0, yaw: 0,
  imu_ok: true, calibrating: false, cal_status: 'idle', cal_samples: 0,
  high_active: false, turn_imu: true,
  battery: { valid: true, voltage: 23.7, percent: 75, held: false, warning: 0, soc_age_s: 0 }
};
function tell(text, error = false) { message = text; messageError = error; render(); }
function fresh() { return online && state !== null && performance.now() - lastFrame < 1500; }
function canMove() { return fresh() && owned && state?.ready && !pending; }
function canRemote() { return fresh() && owned && !state?.fault && !state?.calibrating && !pending; }
function inputs() {
  const down = new Set([...pointers.values(), ...keysDown]);
  return { throttle: Number(down.has('forward')) - Number(down.has('reverse')),
           steer: Number(down.has('left')) - Number(down.has('right')) };
}
function paintInputs() {
  const down = new Set([...pointers.values(), ...keysDown]);
  document.querySelectorAll('.drive').forEach(b => b.classList.toggle('pressed', down.has(b.dataset.dir)));
}
function clearHold() { pointers.clear(); keysDown.clear(); paintInputs(); }
function send(op, data = {}) {
  if (!owned || (!demo && ws?.readyState !== WebSocket.OPEN)) return 0;
  const id = ++seq;
  if (!demo) {
    if (ws.bufferedAmount > 2048) { clearHold(); ws.close(); return 0; }
    ws.send(new URLSearchParams({ op, seq: id, seen: state?.ms || 0, ...data }).toString());
  }
  return id;
}
function sendDrive() {
  if (!owned) return;
  const axes = inputs();
  send('drive', { ...axes, speed: $('remoteSpeed').value });
  if (demo) {
    previewState.state = axes.throttle || axes.steer ? 'REMOTE' : 'IDLE';
    previewState.stationary = previewState.state === 'IDLE';
    previewState.manual = !previewState.stationary;
    previewState.speed = axes.throttle * Number($('remoteSpeed').value);
  }
}
function stop() {
  clearHold(); send('stop'); sendDrive();
  if (demo) { previewState.state = 'IDLE'; previewState.stationary = previewState.ready = true; }
  message = 'Đã dừng.'; messageError = false; render();
}
function rejectWaiting() {
  for (const entry of waiting.values()) entry.reject(new Error('Kết nối bị gián đoạn; lệnh không tự chạy lại.'));
  waiting.clear(); pending = false;
}
function receive(next) {
  if (!next.boot || !Number.isFinite(next.ms)) return;
  if (boot && next.boot !== boot) { clearHold(); rejectWaiting(); }
  boot = next.boot;
  if (next.ms !== lastMs) { lastMs = next.ms; lastFrame = performance.now(); }
  online = true;
  if (next.link_lost) { clearHold(); sendDrive(); }
  if (next.ack && waiting.has(next.ack)) {
    const entry = waiting.get(next.ack); waiting.delete(next.ack);
    if (next.ack_code === 'ok') entry.resolve();
    else entry.reject(new Error(messages[next.ack_code] || 'Chưa thể thực hiện lệnh này.'));
  }
  if (next.cal_status === 'complete' && state?.cal_status !== 'complete') {
    message = 'Hiệu chỉnh IMU hoàn tất. Có thể dùng phần Đặt lệnh.'; messageError = false;
  } else if (next.cal_status === 'failed' && state?.cal_status !== 'failed') {
    message = 'Chưa đủ mẫu IMU ổn định. Đặt xe đứng yên rồi hiệu chỉnh lại.'; messageError = true;
  }
  state = next; render();
}
// Read-only observation never opens a control socket or claims the robot.
function pauseStatus() {
  statusEpoch++; clearTimeout(statusTimer); statusTimer = null;
  statusRequest?.abort(); statusRequest = null;
}
function watchStatus() {
  pauseStatus();
  if (demo || document.hidden || pageLeaving || owned || (ws && ws.readyState <= WebSocket.OPEN)) return;
  pollStatus(statusEpoch);
}
async function pollStatus(epoch) {
  if (epoch !== statusEpoch || document.hidden || pageLeaving || owned) return;
  const request = new AbortController(); statusRequest = request;
  const timeout = setTimeout(() => request.abort(), 1200);
  try {
    const response = await fetch('/api/state', { cache: 'no-store', signal: request.signal });
    if (!response.ok) throw new Error('State unavailable');
    const next = await response.json();
    if (epoch === statusEpoch && !owned && !document.hidden && !pageLeaving) receive(next);
  } catch {
    // Connectivity is separate from ownership; stale readings become unknown.
  } finally {
    clearTimeout(timeout);
    if (epoch === statusEpoch) {
      statusRequest = null; render();
      statusTimer = setTimeout(() => pollStatus(epoch), 500);
    }
  }
}
function openLink() {
  clearTimeout(retry);
  if (!wanted || document.hidden) return;
  pauseStatus();
  if (demo) { owned = online = true; return; }
  const socket = new WebSocket(`ws://${location.host}/ws`); ws = socket;
  socket.onopen = () => { if (ws !== socket || !wanted) { socket.close(); return; } owned = true; seq = 0; lastFrame = performance.now(); lastMs = null; clearHold(); message = ''; messageError = false; render(); };
  socket.onmessage = event => { if (ws !== socket) return; let next; try { next = JSON.parse(event.data); } catch { return; } receive(next); };
  socket.onerror = () => {};
  socket.onclose = () => {
    if (ws !== socket) return;
    owned = false; clearHold(); rejectWaiting(); watchStatus();
    if (wanted) { message = 'Đang kết nối lại với robot…'; messageError = false; retry = setTimeout(openLink, 600); }
    render();
  };
}
function action(op, data, success) {
  if (!canMove() || pending) return;
  clearHold(); pending = true; render();
  const id = send(op, data);
  if (!id) { pending = false; return; }
  if (demo) {
    pending = false; previewState.state = 'MOVE'; previewState.ready = false; previewState.stationary = false; tell(success); return;
  }
  waiting.set(id, {
    resolve: () => { pending = false; tell(success); },
    reject: error => { pending = false; tell(error.message, true); }
  });
}
function render() {
  const connected = fresh(); const running = state && ['MOVE', 'SPEED', 'STOPPING', 'REMOTE'].includes(state.state);
  $('linkDot').classList.toggle('online', connected);
  $('linkText').textContent = demo ? 'Bản xem trước' : connected ? 'Wi-Fi trực tiếp' : 'Chưa có kết nối';
  $('connect').disabled = false;
  $('connect').innerHTML = wanted ? 'Ngắt kết nối <span>↗</span>' : 'Kết nối robot <span>↗</span>';
  $('connectHint').textContent = wanted ? connected && owned ? 'Đã kết nối điều khiển. Chạm nút để điều khiển.' : 'Đang kết nối lại điều khiển…' : connected ? 'Đang theo dõi trạng thái. Kết nối robot để điều khiển.' : 'Kết nối Wi-Fi DUAL-ROBOT; trạng thái sẽ tự cập nhật.';
  $('speedValue').textContent = connected ? format(Math.abs(state?.speed)) : '—';
  $('distanceValue').textContent = connected ? format(state?.distance, 3) : '—';
  $('yawValue').textContent = connected && state?.imu_ok ? format(state.yaw, 1) : '—';
  $('speedBar').style.width = connected ? `${Math.min(100, Math.abs(state?.speed || 0) / .6 * 100)}%` : '0%';
  $('headingArrow').style.transform = `rotate(${connected && state?.imu_ok ? -state.yaw : 0}deg)`;
  $('targetNote').textContent = connected && running ? `Đích encoder ${format(state.target)} m` : 'Ước lượng từ encoder';
  $('imuState').textContent = !connected ? 'Chưa có dữ liệu' : state.calibrating ? 'Đang hiệu chỉnh…' : state.imu_ok ? 'Dữ liệu hợp lệ' : 'Cần hiệu chỉnh';
  const motion = !connected ? 'Mất kết nối' : state.fault ? `Lỗi motor #${state.fault}` : state.calibrating ? 'Hiệu chỉnh IMU' : running ? state.turning ? 'Đang quay tại chỗ' : 'Đang di chuyển' : state.stationary ? 'Đã đứng yên' : 'Đang chờ đứng yên';
  $('motionState').textContent = motion;
  $('dockState').textContent = connected ? motion.toLocaleUpperCase('vi') : wanted ? 'ĐANG KẾT NỐI LẠI' : 'CHƯA KẾT NỐI';
  $('wheelReadings').textContent = connected ? `M1 ${format(state.rpm?.[0], 1)} / M2 ${format(state.rpm?.[1], 1)} RPM` : 'M1 — / M2 — RPM';
  const battery = state?.battery;
  const batteryLive = connected && battery?.valid === true && Number.isFinite(battery.voltage);
  const percentKnown = batteryLive && Number.isFinite(battery.percent);
  const batteryPercent = percentKnown ? Math.max(0, Math.min(100, battery.percent)) : 0;
  $('batteryPercent').textContent = percentKnown ? `${Math.round(batteryPercent)}%` : '—%';
  $('batteryVoltage').textContent = batteryLive ? `${format(battery.voltage)} V` : '— V';
  $('batteryFill').style.width = percentKnown ? `${batteryPercent}%` : '0%';
  const warning = batteryLive ? battery.warning : 0;
  $('batteryStrip').classList.toggle('low', warning === 1);
  $('batteryStrip').classList.toggle('critical', warning === 2);
  const alert = $('batteryAlert');
  alert.hidden = warning !== 1 && warning !== 2;
  alert.classList.toggle('critical', warning === 2);
  const alertText = warning === 2 ? 'Pin rất yếu — cần sạc pin. Mức pin ước tính khi nghỉ đã xuống 10% hoặc thấp hơn.' : warning === 1 ? 'Pin yếu — nên sạc pin. Mức pin ước tính khi nghỉ đã xuống 20% hoặc thấp hơn.' : '';
  if (alert.textContent !== alertText) alert.textContent = alertText;
  $('calHint').textContent = state?.calibrating ? `Giữ xe đứng yên · ${state.cal_samples || 0}/300 mẫu` : state?.imu_ok ? 'IMU sẵn sàng. Quay có bù góc; đi thẳng HIGH có giữ hướng.' : 'Để xe đứng yên và hiệu chỉnh IMU trước khi chạy.';
  $('calibrate').disabled = !connected || !owned || !state?.stationary || running || state?.calibrating || pending;
  $('clearFault').hidden = !state?.fault;
  $('clearFault').disabled = !connected || !owned || !state?.stationary || running;
  document.querySelectorAll('.launch').forEach((b) => { b.disabled = !canMove() || pointers.size > 0 || keysDown.size > 0; });
  document.querySelectorAll('.drive').forEach((b) => { b.disabled = !canRemote(); });
  $('remoteSpeed').disabled = false;
  $('moveFields').disabled = !!running || pending;
  $('turnFields').disabled = !!running || pending;
  const notice = $('notice');
  notice.classList.toggle('error', messageError && !demo); notice.classList.toggle('demo', demo);
  notice.textContent = demo ? 'XEM TRƯỚC OFFLINE · Không kết nối phần cứng, không phát lệnh motor.' : message || (connected && owned ? 'Remote: giữ tiến/lùi và thêm trái/phải để rẽ. Thả tay để dừng.' : wanted ? 'Đang kết nối lại điều khiển. Khi kết nối xong, chạm lại nút để chạy.' : connected ? 'Trạng thái tự cập nhật. Kết nối robot để gửi lệnh điều khiển.' : 'Đang chờ dữ liệu. Kết nối điện thoại vào Wi-Fi DUAL-ROBOT.');
}

$('connect').addEventListener('click', () => {
  wanted = !wanted; clearHold(); message = ''; messageError = false;
  if (wanted) openLink();
  else { send('stop'); owned = false; clearTimeout(retry); ws?.close(); rejectWaiting(); watchStatus(); }
  render();
});
document.querySelectorAll('.drive').forEach(button => {
  button.addEventListener('pointerdown', event => {
    if (event.button !== 0 && event.pointerType === 'mouse') return;
    if (!canRemote()) return;
    event.preventDefault(); button.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, button.dataset.dir); paintInputs(); sendDrive();
  });
  button.addEventListener('pointermove', event => {
    if (!pointers.has(event.pointerId)) return;
    const target = document.elementFromPoint(event.clientX, event.clientY)?.closest('.drive');
    if (target && target.dataset.dir !== pointers.get(event.pointerId)) {
      pointers.set(event.pointerId, target.dataset.dir); paintInputs(); sendDrive();
    }
  });
  button.addEventListener('lostpointercapture', event => {
    if (pointers.delete(event.pointerId)) { paintInputs(); sendDrive(); }
  });
  button.addEventListener('contextmenu', event => event.preventDefault());
});
for (const name of ['pointerup', 'pointercancel']) window.addEventListener(name, event => {
  if (pointers.delete(event.pointerId)) { paintInputs(); sendDrive(); }
}, true);
const keys = { ArrowUp: 'forward', ArrowDown: 'reverse', ArrowLeft: 'left', ArrowRight: 'right' };
window.addEventListener('keydown', event => {
  if (event.key === 'Escape' || (event.code === 'Space' && !/INPUT|SELECT|BUTTON/.test(event.target.tagName))) { event.preventDefault(); stop(); return; }
  if (/INPUT|SELECT|TEXTAREA/.test(event.target.tagName) || $('calDialog').open || !keys[event.key]) return;
  event.preventDefault(); if (!canRemote()) return;
  keysDown.add(keys[event.key]); paintInputs(); sendDrive();
});
window.addEventListener('keyup', event => { if (keysDown.delete(keys[event.key])) { paintInputs(); sendDrive(); } });
window.addEventListener('blur', () => { clearHold(); send('stop'); });
document.addEventListener('visibilitychange', () => {
  clearHold();
  if (document.hidden) { send('stop'); pauseStatus(); }
  else if (wanted && !owned) openLink();
  else if (!owned) watchStatus();
});
window.addEventListener('pagehide', () => { pageLeaving = true; pauseStatus(); clearHold(); send('stop'); wanted = false; ws?.close(); });
window.addEventListener('pageshow', () => { pageLeaving = false; if (!owned) watchStatus(); });
$('stop').addEventListener('click', stop);
$('remoteSpeed').addEventListener('input', () => {
  $('remoteSpeedValue').innerHTML = `${format(+$('remoteSpeed').value)} <small>m/s</small>`; sendDrive();
});
document.querySelectorAll('[data-tab]').forEach(button => button.addEventListener('click', () => {
  clearHold(); sendDrive(); document.body.dataset.tab = button.dataset.tab;
  document.querySelectorAll('[data-tab]').forEach(other => { const active = other === button; other.classList.toggle('selected', active); other.setAttribute('aria-selected', active); });
}));
document.querySelectorAll('[data-layout]').forEach(button => button.addEventListener('click', () => {
  $('directionPad').dataset.layout = button.dataset.layout;
  document.querySelectorAll('[data-layout]').forEach(b => { b.classList.toggle('selected', b === button); b.setAttribute('aria-pressed', b === button); });
  try { localStorage.setItem('dualRemoteLayout', button.dataset.layout); } catch {}
}));
try { const layout = localStorage.getItem('dualRemoteLayout'); if (layout === 'split') document.querySelector('[data-layout="split"]').click(); } catch {}
document.querySelectorAll('[data-mission]').forEach(button => button.addEventListener('click', () => {
  $('moveForm').hidden = button.dataset.mission !== 'move'; $('turnForm').hidden = button.dataset.mission !== 'turn';
  document.querySelectorAll('[data-mission]').forEach(b => b.classList.toggle('selected', b === button));
}));
document.querySelectorAll('[data-speed]').forEach(button => button.addEventListener('click', () => {
  $('moveSpeed').value = button.dataset.speed;
  document.querySelectorAll('[data-speed]').forEach(b => b.classList.toggle('selected', b === button));
}));
document.querySelectorAll('[data-angle]').forEach(button => button.addEventListener('click', () => {
  $('angle').value = button.dataset.angle; $('turnSeconds').value = ({ 90: 4, 180: 6, 360: 12 })[button.dataset.angle];
  document.querySelectorAll('[data-angle]').forEach(b => b.classList.toggle('selected', b === button));
}));
$('moveForm').addEventListener('submit', event => { event.preventDefault(); action('move', { dir: $('moveDirection').value, distance: $('distance').value, speed: $('moveSpeed').value }, 'Robot đang thực hiện hành trình.'); });
$('turnForm').addEventListener('submit', event => { event.preventDefault(); action('turn', { dir: $('turnDirection').value, angle: $('angle').value, seconds: $('turnSeconds').value }, 'Robot đang quay theo góc đặt.'); });
$('calibrate').addEventListener('click', () => $('calDialog').showModal());
$('cancelCal').addEventListener('click', () => $('calDialog').close());
$('confirmCal').addEventListener('click', () => {
  $('calDialog').close(); if (!fresh() || !owned) return;
  clearHold(); pending = true; const id = send('calibrate');
  if (demo) { pending = false; tell('Xem trước: quá trình hiệu chỉnh chỉ thực hiện trên robot.'); return; }
  if (!id) { pending = false; return; }
  waiting.set(id, { resolve: () => { pending = false; tell('Đang hiệu chỉnh IMU. Giữ robot đứng yên.'); }, reject: error => { pending = false; tell(error.message, true); } }); render();
});
$('clearFault').addEventListener('click', () => { send('clear'); message = ''; render(); });
setInterval(() => {
  if (demo) { previewState.ms = Math.round(performance.now()); receive({ ...previewState }); }
  if (!demo && owned) {
    if (performance.now() - lastFrame > 350 && (pointers.size || keysDown.size)) clearHold();
    if (lastFrame && performance.now() - lastFrame > 1500) { clearHold(); ws?.close(); return; }
    sendDrive();
  }
  if (!fresh()) render();
}, 100);
render();
watchStatus();
