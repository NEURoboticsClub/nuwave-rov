// Exercise the production command helper without the dashboard's rendering loop.
const { readFileSync } = require('node:fs');
const { resolve } = require('node:path');
const { runInNewContext } = require('node:vm');
const { test } = require('node:test');
const assert = require('node:assert/strict');
const path = resolve(__dirname, '../static/build_dashboard.js');
const source = readFileSync(path, 'utf8');
const helper = source.slice(source.indexOf('function publishMessage('),
    source.indexOf('function downloadAllCameraScreenshots('));

for (const [name, state, expected] of [
    ['connecting', 0, false], ['open', 1, true], ['closing', 2, false],
    ['closed', 3, false], ['missing', null, false], ['invalid sender', 1, false],
    ['missing state', undefined, false], ['send race', 1, false],
]) {
    test(name, () => {
        const sent = [];
        const socket = state === null ? undefined : { readyState: state, send(payload) {
            if (state !== 1 || name === 'send race') throw new Error('not connected');
            sent.push(payload);
        } };
        if (name === 'invalid sender') socket.send = null;
        const context = { window: { ws: socket }, WebSocket: { OPEN: 1 }, console: { warn() {} } };
        runInNewContext(helper, context, { filename: path });
        let result;
        assert.doesNotThrow(() => { result = context.publishMessage('/gui_buttons/expo_enabled', true); });
        assert.equal(result, expected);
        assert.equal(sent.length, Number(expected));
        if (expected) assert.deepEqual(JSON.parse(sent[0]), { topic: '/gui_buttons/expo_enabled', data: true });
    });
}
