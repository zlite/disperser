#pragma once

#include <Arduino.h>

const char DISPERSER_WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Bug Disperser</title>
  <style>
    :root { color-scheme: dark; font-family: Inter, system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #07111d; color: #eaf2f8; }
    main { width: min(760px, 100%); margin: auto; padding: 22px; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 16px; }
    h1 { margin: 0; font-size: 1.55rem; }
    .pill { padding: 7px 11px; border-radius: 999px; background: #263647; font-weight: 700; }
    .pill.run { background: #075b4c; color: #8fffe7; }
    .pill.error { background: #681f2a; color: #ffc4cc; }
    .card { margin-top: 16px; padding: 17px; border: 1px solid #26394c; border-radius: 14px; background: #101e2c; box-shadow: 0 8px 28px #0004; }
    .metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .metric { padding: 12px; border-radius: 10px; background: #0a1622; }
    .label { color: #91a7ba; font-size: .74rem; letter-spacing: .06em; text-transform: uppercase; }
    .value { margin-top: 4px; font-size: 1.08rem; font-weight: 700; }
    .buttons { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
    button { min-height: 48px; border: 0; border-radius: 10px; background: #1677d2; color: white; font-weight: 800; cursor: pointer; }
    button.stop { background: #c23b4a; }
    button:disabled { cursor: not-allowed; opacity: .45; }
    .control { margin: 18px 0; }
    .control-head { display: flex; justify-content: space-between; margin-bottom: 8px; }
    input[type=range], select { width: 100%; accent-color: #2db5e8; }
    select { padding: 11px; border: 1px solid #36516a; border-radius: 9px; background: #0a1622; color: #eaf2f8; }
    .hint, #notice { color: #91a7ba; font-size: .86rem; }
    #notice { min-height: 1.3em; margin-top: 12px; }
    @media (max-width: 520px) { .metrics { grid-template-columns: 1fr; } .buttons { grid-template-columns: 1fr; } }
  </style>
</head>
<body><main>
  <header><h1>Bug Disperser</h1><div id="state" class="pill">Connecting</div></header>

  <section class="card metrics">
    <div class="metric"><div class="label">Address</div><div id="address" class="value">disperser.local</div></div>
    <div class="metric"><div class="label">Limits L0 / L1 / L2</div><div id="limit" class="value">—</div></div>
    <div class="metric"><div class="label">Position</div><div id="position" class="value">—</div></div>
    <div class="metric"><div class="label">Run time</div><div id="elapsed" class="value">0 / 0 s</div></div>
  </section>

  <section class="card">
    <div class="buttons">
      <button id="home" type="button" data-command="home">Home Z</button>
      <button id="start" type="button" data-command="start">Start</button>
      <button id="pause" type="button" data-command="pause">Pause</button>
    </div>
    <div id="notice"></div>
  </section>

  <section class="card">
    <div class="control">
      <div class="control-head"><label for="motion">Motion type</label></div>
      <select id="motion" onchange="saveSettings()">
        <option value="circular">Circular</option>
        <option value="back-and-forth">Back-and-forth</option>
        <option value="side-to-side">Side-to-side</option>
        <option value="combination">Combination</option>
      </select>
    </div>
    <div class="control">
      <div class="control-head"><label for="speed">XY speed</label><strong><span id="speedOut">2.0</span> mm/s</strong></div>
      <input id="speed" type="range" min="0.5" max="10" step="0.5" oninput="showValues()" onchange="saveSettings()">
    </div>
    <div class="control">
      <div class="control-head"><label for="duration">Duration</label><strong><span id="durationOut">30</span> s</strong></div>
      <input id="duration" type="range" min="5" max="300" step="5" oninput="showValues()" onchange="saveSettings()">
    </div>
    <div class="control">
      <div class="control-head"><label for="size">Radius / travel</label><strong><span id="sizeOut">2.0</span> mm</strong></div>
      <input id="size" type="range" min="0.5" max="10" step="0.5" oninput="showValues()" onchange="saveSettings()">
    </div>
    <div class="hint">Changes apply on the next Start. Combination runs circles and both linear patterns.</div>
  </section>
</main>
<script>
let loaded = false;
let busy = false;
const $ = id => document.getElementById(id);
function showValues() {
  $('speedOut').textContent = Number($('speed').value).toFixed(1);
  $('durationOut').textContent = $('duration').value;
  $('sizeOut').textContent = Number($('size').value).toFixed(1);
}
async function sendCommand(name, button) {
  if (busy) return;
  busy = true;
  const originalText = button.textContent;
  button.textContent = 'Sending…';
  $('notice').textContent = 'Sending command…';
  try {
    const response = await fetch('/api/' + name, { method: 'POST' });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || ('HTTP ' + response.status));
    $('notice').textContent = data.message || 'Command accepted';
    await refresh();
  } catch (error) { $('notice').textContent = error.message; }
  button.textContent = originalText;
  busy = false;
}
async function saveSettings() {
  showValues();
  const query = new URLSearchParams({
    motion: $('motion').value,
    speed: $('speed').value,
    duration: $('duration').value,
    size: $('size').value
  });
  try {
    const response = await fetch('/api/settings?' + query, { method: 'POST' });
    if (!response.ok) throw new Error('Settings rejected');
    $('notice').textContent = 'Settings saved for next run';
  } catch (error) { $('notice').textContent = error.message; }
}
async function refresh() {
  try {
    const response = await fetch('/api/status');
    const data = await response.json();
    $('state').textContent = data.state;
    $('state').className = 'pill ' + (data.error ? 'error' : (data.moving ? 'run' : ''));
    $('address').textContent = 'disperser.local · ' + data.ip;
    const limitText = value => value < 0 ? '?' : (value ? 'HIT' : 'open');
    $('limit').textContent = `Z ${limitText(data.limitZ)} · X ${limitText(data.limitX)} · Y ${limitText(data.limitY)}`;
    $('position').textContent = `X ${data.x.toFixed(1)} · Y ${data.y.toFixed(1)} · Z ${data.z.toFixed(1)} mm`;
    $('elapsed').textContent = `${data.elapsed} / ${data.activeDuration || data.duration} s`;
    $('start').textContent = data.moving ? 'Stop' : 'Start';
    $('start').className = data.moving ? 'stop' : '';
    $('pause').textContent = data.paused ? 'Resume' : 'Pause';
    $('home').disabled = data.moving;
    $('pause').disabled = !data.moving || data.stopping;
    if (!loaded) {
      $('motion').value = data.motion;
      $('speed').value = data.speed;
      $('duration').value = data.duration;
      $('size').value = data.size;
      showValues();
      loaded = true;
    }
  } catch (_) {
    $('state').textContent = 'Offline';
    $('state').className = 'pill error';
  }
}
document.querySelectorAll('button[data-command]').forEach(button => {
  button.addEventListener('click', event => {
    event.preventDefault();
    sendCommand(button.dataset.command, button);
  });
});
setInterval(refresh, 500);
refresh();
</script></body></html>
)HTML";
