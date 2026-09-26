// Test the actual inline portal script without a board or browser connection.
// node water_sensor_portal_test.cjs <water_level_st7789.ino>
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(process.argv[2], 'utf8');
const lines = source.split(/\r?\n/);
const start = lines.findIndex(line => line.includes('let lastUpdate=0;'));
assert(start >= 0, 'Portal script must exist');
let script = '';
for (const line of lines.slice(start)) {
  const match = line.match(/h \+= F\((".*")\);/);
  assert(match, 'Expected an inline script string');
  script += JSON.parse(match[1]);
  if (script.includes('</script>')) break;
}
script = script.split('</script>')[0];
const fields = Object.fromEntries(['w', 'raw', 'diag', 'ps', 'er'].map(id => [id, {textContent: ''}]));
const timers = [];
let now = 10000;
let response = {};
let fail = false;
const context = vm.createContext({
  document: {getElementById: id => { assert(fields[id], `Missing field ${id}`); return fields[id]; }},
  window: {}, Date: {now: () => now}, AbortSignal: {timeout: () => undefined},
  setInterval: (fn, interval) => timers.push({fn, interval}),
  fetch: async () => { if (fail) throw Error('Offline'); return {ok: true, json: async () => response}; }
});
vm.runInContext(script, context);
(async () => {
  response = {sensor_ok: false, monitor_configured: false, raw_distance_cm: 80,
    echo_count: 5, sensor_status: 'monitor_not_configured', coil_voltage: 'unknown', alarm: 'monitor_not_configured'};
  await context.u();
  assert.equal(fields.w.textContent, 'Set tank height');
  assert.equal(fields.raw.textContent, '80.0 cm (5/5 echoes)');
  assert.match(fields.diag.textContent, /Save actual tank height/);
  response = {...response, sensor_ok: true, monitor_configured: true, water: 40, sensor_status: 'ok', alarm: ''};
  await context.u();
  assert.equal(fields.w.textContent, '40.0 cm');
  for (const status of ['no_echo', 'unstable_echo', 'below_blind_zone', 'out_of_range', 'outside_tank_range', 'sample_stale']) {
    response = {...response, sensor_ok: false, sensor_status: status};
    await context.u();
    assert.equal(fields.w.textContent, 'Unavailable');
    assert.notEqual(fields.diag.textContent, 'Unknown sensor status');
  }
  response.raw_distance_cm = -1;
  await context.u();
  assert.equal(fields.raw.textContent, 'No fresh echo');
  fail = true;
  await context.u();
  assert.equal(fields.raw.textContent, 'Unknown');
  assert.equal(fields.w.textContent, 'Unknown');
  fail = false;
  await context.u();
  now += 11000;
  timers.find(timer => timer.interval === 1000).fn();
  assert.equal(fields.raw.textContent, 'Unknown');
  assert.equal(fields.ps.textContent, 'Unknown / offline');
  console.log('SENSOR_PORTAL_TEST_OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
