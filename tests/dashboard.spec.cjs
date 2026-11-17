// Offline browser/transport tests. No hardware or real robot network is used.
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const { wsServer: WebSocketServer } = require(process.env.PLAYWRIGHT_UTILS || 'playwright-core/lib/utilsBundle.js');
const http = require('node:http'), fs = require('node:fs'), path = require('node:path');
const assert = require('node:assert/strict');
const { pathToFileURL } = require('node:url');
const root = path.resolve(__dirname, '..'), evidence = path.join(root, 'docs/wifi-validation');
const mime = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml', '.woff2': 'font/woff2' };
let server, sockets, browser, timer, checks = 0, connections = 0, stateReads = 0;
const check = (value, note) => { assert.ok(value, note); checks++; };
async function main() {
 const commands = [], errors = []; let freeze = false, delayAck = 0, active, stateUnavailable = false;
 let state = { boot: 'OFFLINE', ms: 1000, state: 'IDLE', fault: 0, ready: false, remote_ready: true, stationary: true,
  owned: true, manual: false, run: 0, speed: 0, rpm: [0,0], sp: [0,0], distance: 0, target: 0, yaw: 0,
  imu_ok: false, cal_status: 'idle', cal_samples: 0, calibrating: false, ack: 0, ack_code: 'ok',
  battery: { valid: true, voltage: 24.1, percent: 85, held: false, warning: 0, soc_age_s: 0 } };
 server = http.createServer((req,res) => {
  const pathname = new URL(req.url,'http://localhost').pathname;
  if (pathname === '/api/state' && req.method === 'GET') {
   stateReads++;
   if (stateUnavailable) { res.writeHead(503).end();return; }
   res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});res.end(JSON.stringify(state));return;
  }
  if (pathname.startsWith('/api/')) { errors.push('Unexpected HTTP control');res.writeHead(400).end();return; }
  const file = path.join(root,'dashboard',pathname === '/'?'index.html':pathname);
  if (!file.startsWith(path.join(root,'dashboard')) || !fs.existsSync(file)) {res.writeHead(404).end();return;}
  res.writeHead(200,{'Content-Type':mime[path.extname(file)]||'text/plain','Content-Security-Policy':"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; font-src 'self'; connect-src 'self' ws:; frame-ancestors 'none'"});res.end(fs.readFileSync(file));
 });
 await new Promise(resolve => server.listen(0,'127.0.0.1',resolve));
 sockets = new WebSocketServer({server,path:'/ws'});
 sockets.on('connection', socket => {
  active=socket;connections++;state.ack=0;
  socket.on('message', bytes => {
   const data=Object.fromEntries(new URLSearchParams(bytes.toString()));commands.push(data);
   if(data.op==='drive') {
    const moving=Number(data.throttle)||Number(data.steer);
    if(moving){state.state='REMOTE';state.manual=true;state.stationary=false;state.ready=false;}
    else if(state.manual){state.state='IDLE';state.manual=false;state.stationary=true;state.ready=state.imu_ok;}
   } else {
    const ack=()=>{state.ack=Number(data.seq);state.ack_code='ok';};
    if(data.op==='stop'){state.state='IDLE';state.manual=false;state.stationary=true;state.ready=state.imu_ok;ack();}
    if(data.op==='move'||data.op==='turn'){state.state='MOVE';state.ready=false;state.stationary=false;state.run++;ack();}
    if(data.op==='calibrate'){state.calibrating=true;state.cal_status='warming';state.cal_samples=0;state.ready=false;if(delayAck)setTimeout(ack,delayAck);else ack();}
    if(data.op==='clear'){state.fault=0;ack();}
   }
  });
  socket.on('close',()=>{state.state='IDLE';state.manual=false;state.stationary=true;});
 });
 timer=setInterval(()=>{
  state.ms+=100;
  if(state.calibrating){state.cal_samples=Math.min(300,state.cal_samples+10);state.cal_status='sampling';if(state.cal_samples===300){state.calibrating=false;state.cal_status='complete';state.imu_ok=state.ready=true;}}
  if(!freeze&&active?.readyState===1)active.send(JSON.stringify(state));
 },100);
 browser=await chromium.launch({executablePath:process.env.BROWSER_EXE,headless:true});
 const ctx=await browser.newContext({viewport:{width:1280,height:1100},hasTouch:true});
 const page=await ctx.newPage();page.on('pageerror',e=>errors.push(e.message));
 const base=`http://127.0.0.1:${server.address().port}`;
 await page.goto(base);await page.waitForTimeout(150);
 check(connections===0&&commands.length===0,'load never claims or moves automatically');
 check(stateReads>0,'state observation starts automatically using read-only GET');
 check(await page.locator('#batteryPercent').textContent()==='85%','battery visible before control connection');
 check((await page.locator('#batteryVoltage').textContent()).includes('24,10'),'voltage visible before control connection');
 check(await page.locator('#batteryState, #batteryNote, .battery-detail').count()===0,'battery detail lines removed');
 check(await page.locator('[data-dir=forward]').isDisabled(),'remote disabled before connect');
 check(await page.locator('#padStop').count()===0,'center stop removed');
 check(await page.locator('#stop').isVisible(),'separate STOP kept');
 await page.locator('#connect').click();await page.waitForTimeout(250);
 check(connections===1,'one persistent socket');
 check(await page.locator('[data-dir=forward]').isEnabled(),'remote works without IMU');
 check(await page.locator('#moveForm .launch').isDisabled(),'setpoint waits for IMU');
 check((await page.locator('#batteryPercent').textContent())==='85%','live battery percent');
 const readsWhenControlled=stateReads;await page.waitForTimeout(650);
 check(stateReads===readsWhenControlled,'control WebSocket avoids duplicate HTTP state polling');
 const up=page.locator('[data-dir=forward]');await up.scrollIntoViewIfNeeded();
 await page.keyboard.down('ArrowUp');await page.waitForTimeout(180);
 check(commands.some(x=>x.op==='drive'&&x.throttle==='1'&&x.steer==='0'),'forward input');
 check(await page.locator('[data-dir=left]').isEnabled(),'steering remains available while moving');
 await page.keyboard.down('ArrowLeft');await page.waitForTimeout(180);
 check(commands.some(x=>x.op==='drive'&&x.throttle==='1'&&x.steer==='1'),'forward plus left chord');
 await page.locator('#remoteSpeed').evaluate(el=>{el.value='0.5';el.dispatchEvent(new Event('input',{bubbles:true}));});await page.waitForTimeout(120);
 check(commands.some(x=>x.op==='drive'&&x.speed==='0.5'&&x.throttle==='1'&&x.steer==='1'),'speed can change while steering');
 await page.keyboard.up('ArrowLeft');await page.waitForTimeout(120);
 check(commands.at(-1).throttle==='1'&&commands.at(-1).steer==='0','release steering keeps forward');
 await page.keyboard.up('ArrowUp');await page.waitForTimeout(180);
 check(commands.at(-1).throttle==='0'&&commands.at(-1).steer==='0','all keys released sends neutral');
 for(let n=0;n<5;n++){await page.keyboard.down('ArrowDown');await page.waitForTimeout(70);await page.keyboard.up('ArrowDown');await page.waitForTimeout(70);}
 check(connections===1,'repeated drive/release never reconnects');
 await page.locator('[data-layout=split]').click();
 check(await page.locator('#directionPad').getAttribute('data-layout')==='split','split layout selectable');
 const ub=await up.boundingBox(),lb=await page.locator('[data-dir=left]').boundingBox();
 check(ub.x<lb.x,'split throttle left, steering right');
 const cdp=await ctx.newCDPSession(page);
 await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:ub.x+30,y:ub.y+30,id:1}]});await page.waitForTimeout(100);
 await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:ub.x+30,y:ub.y+30,id:1},{x:lb.x+30,y:lb.y+30,id:2}]});await page.waitForTimeout(150);
 check(commands.at(-1).throttle==='1'&&commands.at(-1).steer==='1','two real touch pointers combine');
 await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[{x:lb.x+30,y:lb.y+30,id:2}]});await page.waitForTimeout(120);
 check(commands.at(-1).throttle==='1'&&commands.at(-1).steer==='0','one finger release preserves other: '+JSON.stringify(commands.slice(-3)));
 await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await page.waitForTimeout(150);
 check(commands.at(-1).throttle==='0'&&commands.at(-1).steer==='0','touch release neutral');
 await page.locator('#distance').focus();let before=commands.filter(x=>Number(x.throttle)||Number(x.steer)).length;await page.keyboard.press('ArrowUp');
 check(commands.filter(x=>Number(x.throttle)||Number(x.steer)).length===before,'form arrows never drive');
 // Same live socket survives window blur; held input is stopped, no re-arm dialog.
 await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await page.waitForTimeout(200);
 check(connections===1&&await up.isEnabled(),'blur does not expire connection');
 await page.keyboard.press('Escape');await page.waitForTimeout(150);
 check(commands.some(x=>x.op==='stop'),'Esc sends independent stop');
 await page.locator('#calibrate').click();delayAck=650;await page.locator('#confirmCal').click();await page.waitForTimeout(900);
 check(!/aborted|quá hạn|hết hạn/.test(await page.locator('#notice').textContent()),'delayed calibration ACK has no HTTP timeout');
 check((await page.locator('#calHint').textContent()).includes('/300 mẫu'),'calibration progress shown');
 await page.waitForFunction(() => document.getElementById('notice').textContent.includes('hoàn tất'), null, {timeout:8000});
 check(state.cal_samples===300&&state.imu_ok,'300 samples before complete');
 check((await page.locator('#notice').textContent()).includes('hoàn tất'),'completion confirmed from device state');
 await page.locator('#moveDirection').selectOption('reverse');await page.locator('#distance').fill('1.5');await page.locator('#moveSpeed').fill('0.4');
 await page.locator('#moveForm .launch').click();await page.waitForTimeout(250);
 check(commands.some(x=>x.op==='move'&&x.dir==='reverse'&&x.distance==='1.5'&&x.speed==='0.4'),'setpoint data unchanged');
 await page.locator('#stop').click();await page.waitForTimeout(250);
 await page.locator('[data-mission=turn]').click();await page.locator('[data-angle="360"]').click();await page.locator('#turnDirection').selectOption('right');
 await page.locator('#turnForm .launch').click();await page.waitForTimeout(250);
 check(commands.some(x=>x.op==='turn'&&x.dir==='right'&&x.angle==='360'&&x.seconds==='12'),'turn form unchanged');
 await page.locator('#stop').click();await page.waitForTimeout(200);
 // Drop the socket while a key is held: auto reconnect, but no replayed movement.
 await up.focus();await page.keyboard.down('ArrowUp');await page.waitForTimeout(150);const con=connections;
 active.terminate();await page.waitForTimeout(1200);
 check(connections>con&&await up.isEnabled(),'automatic reconnection without connect button');
 check(commands.at(-1).throttle==='0'&&commands.at(-1).steer==='0','held motion not replayed after reconnect');
 await page.keyboard.up('ArrowUp');await page.waitForTimeout(100);
 // Status pause clears held intent. It must not create an expiry popup.
 await up.focus();await page.keyboard.down('ArrowDown');await page.waitForTimeout(150);freeze=true;await page.waitForTimeout(550);freeze=false;await page.waitForTimeout(200);
 check(commands.at(-1).throttle==='0','stale feedback clears held intent');await page.keyboard.up('ArrowDown');
 check(!/hết hạn|chưa xác nhận|quá hạn/.test(await page.locator('#notice').textContent()),'no expiry/ACK protection banners');
 state.battery.warning=1;state.battery.percent=19;await page.waitForTimeout(150);check(await page.locator('#batteryAlert').isVisible(),'low battery warning preserved');
 state.battery.warning=2;await page.waitForTimeout(150);check((await page.locator('#batteryAlert').textContent()).includes('rất yếu'),'critical battery warning preserved');
 state.battery.valid=false;await page.waitForTimeout(150);check(await page.locator('#batteryPercent').textContent()==='—%','invalid battery not fabricated');
 state.fault=3;await page.waitForTimeout(150);check(await up.isDisabled(),'physical motor fault still stops admission');state.fault=0;
 // A second page observes without claiming or disrupting the control socket.
 state.battery.valid=true;state.battery.percent=78;
 const observer=await ctx.newPage();observer.on('pageerror',e=>errors.push(e.message));const ownedConnections=connections;
 await observer.goto(base);await observer.waitForTimeout(650);
 check(connections===ownedConnections,'observer never claims a control WebSocket');
 check(await observer.locator('#batteryPercent').textContent()==='78%','second device reads battery while another controls');
 check(await observer.locator('[data-dir=forward]').isDisabled(),'observing another controller never grants local motion');
 await observer.close();
 // Explicit disconnect releases control, but leaves battery/state observation live.
 await page.locator('#connect').click();await page.waitForTimeout(250);
 const releasedCommands=commands.length, releasedConnections=connections, readsBefore=stateReads;
 state.battery.percent=71;state.battery.voltage=23.2;await page.waitForTimeout(700);
 check(stateReads>readsBefore&&await page.locator('#batteryPercent').textContent()==='71%','battery keeps updating after explicit control disconnect');
 check((await page.locator('#batteryVoltage').textContent()).includes('23,20'),'disconnected voltage remains live');
 check(commands.length===releasedCommands&&connections===releasedConnections,'observation sends no commands or control reconnect');
 check(await up.isDisabled()&&await page.locator('#calibrate').isDisabled(),'status observation never enables motion or calibration');
 stateUnavailable=true;await page.waitForTimeout(2000);
 check(await page.locator('#batteryPercent').textContent()==='—%'&&await page.locator('#batteryVoltage').textContent()==='— V','lost state link hides stale battery values');
 stateUnavailable=false;await page.waitForTimeout(750);
 check(await page.locator('#batteryPercent').textContent()==='71%','read-only state reconnects automatically without button');
 state.boot='REBOOTED';state.ms=0;state.battery.percent=70;await page.waitForTimeout(650);
 check(await page.locator('#batteryPercent').textContent()==='70%'&&await up.isDisabled(),'robot reboot resumes observation without claiming control');
 check(commands.length===releasedCommands&&connections===releasedConnections,'recovery and reboot never replay commands');
 // Explicit standalone preview captures, not evidence of hardware operation.
 await page.goto(`${base}/?preview=1`);await page.locator('#connect').click();await page.waitForTimeout(200);
 await page.locator('[data-layout=center]').click();await page.screenshot({path:path.join(evidence,'dashboard-desktop.png'),fullPage:true});
 await page.setViewportSize({width:390,height:844});await page.waitForTimeout(100);
 check(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'390px no overflow');
 await page.screenshot({path:path.join(evidence,'dashboard-mobile-remote.png'),fullPage:true});
 await page.locator('[data-layout=split]').click();await page.screenshot({path:path.join(evidence,'dashboard-mobile-split.png'),fullPage:true});
 await page.locator('[data-tab=mission]').click();await page.screenshot({path:path.join(evidence,'dashboard-mobile-setpoint.png'),fullPage:true});
 await page.setViewportSize({width:320,height:700});check(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'320px no overflow');
 const offline=await ctx.newPage(),requests=[];offline.on('request',r=>{if(/^https?:/.test(r.url()))requests.push(r.url());});
 await offline.goto(pathToFileURL(path.join(root,'dashboard/preview.html')).href);await offline.locator('#connect').click();await offline.waitForTimeout(200);
 check((await offline.locator('#notice').textContent()).includes('XEM TRƯỚC')&&requests.length===0,'standalone preview no network');await offline.close();
 check(errors.length===0,errors.join('; '));
 fs.writeFileSync(path.join(evidence,'browser-r4-tests.json'),JSON.stringify({result:'PASS',checks,errors,hardware:false,connections,stateReads,frames:commands.length,commands:commands.filter(x=>x.op!=='drive')},null,2));
 console.log(`STATUS_BROWSER_PASS checks=${checks} frames=${commands.length} connections=${connections} stateReads=${stateReads}`);
}
main().catch(e=>{console.error(e);process.exitCode=1;}).finally(async()=>{clearInterval(timer);if(sockets){for(const s of sockets.clients)s.terminate();sockets.close();}if(browser)await browser.close();if(server)server.close();});
