const dashboard = {
    meters: new Map(),
    sparkline: null,
    cameras: new Map(),
};

function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
}

function setText(node, value) {
    if (!node) {
        return;
    }
    node.textContent = value;
}

function resizeCanvasToDisplaySize(canvas, context) {
    const cssWidth = Math.max(1, Math.floor(canvas.clientWidth || 1));
    const cssHeight = Math.max(1, Math.floor(canvas.clientHeight || 1));
    const dpr = window.devicePixelRatio || 1;
    const pixelWidth = Math.max(1, Math.floor(cssWidth * dpr));
    const pixelHeight = Math.max(1, Math.floor(cssHeight * dpr));

    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        canvas.width = pixelWidth;
        canvas.height = pixelHeight;
    }

    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { width: cssWidth, height: cssHeight };
}

function createPanel(title, topic = "") {
    const panel = document.createElement("section");
    panel.className = "panel";

    const header = document.createElement("div");
    header.className = "panel__header";

    const heading = document.createElement("h2");
    heading.textContent = title;

    const topicEl = document.createElement("span");
    topicEl.className = "panel__topic";
    topicEl.textContent = topic;

    header.append(heading, topicEl);
    panel.appendChild(header);
    document.getElementById("dashboard").appendChild(panel);
    return panel;
}

function registerMeter(options) {
    const row = document.createElement("div");
    row.className = "row";

    const head = document.createElement("div");
    head.className = "row__head";

    const label = document.createElement("span");
    label.className = "row__label";
    label.textContent = options.label;

    const value = document.createElement("span");
    value.className = "row__value";
    value.textContent = options.placeholder || "--";

    head.append(label, value);
    row.appendChild(head);

    let fill = null;
    if (options.bar) {
        const bar = document.createElement("div");
        bar.className = "bar";

        fill = document.createElement("div");
        fill.className = "bar__fill";
        fill.style.setProperty("--value", "0");

        bar.appendChild(fill);
        row.appendChild(bar);
    }

    options.parent.appendChild(row);

    dashboard.meters.set(options.key, {
        row,
        value,
        fill,
        min: options.min,
        max: options.max,
        format: options.format,
        lastSeen: 0,
    });
}

function registerTableMeter(parent, key, label, format) {
    const name = document.createElement("div");
    name.className = "matrix__name";
    name.textContent = label;

    const value = document.createElement("div");
    value.className = "matrix__cell";
    value.textContent = "--";

    parent.append(name, value);

    dashboard.meters.set(key, {
        row: value,
        value,
        fill: null,
        min: 0,
        max: 1,
        format,
        lastSeen: 0,
    });
}

function registerSignedBarMeter(parent, key, label, min = -0.5, max = 0.5) {
    const row = document.createElement("div");
    row.className = "command-row";

    const head = document.createElement("div");
    head.className = "row__head";

    const axis = document.createElement("span");
    axis.className = "row__label";
    axis.textContent = label;

    const value = document.createElement("span");
    value.className = "row__value";
    value.textContent = "0.00";

    head.append(axis, value);
    row.appendChild(head);

    const bar = document.createElement("div");
    bar.className = "signed-bar";

    const negFill = document.createElement("div");
    negFill.className = "signed-bar__neg";

    const posFill = document.createElement("div");
    posFill.className = "signed-bar__pos";

    const center = document.createElement("div");
    center.className = "signed-bar__center";

    bar.append(negFill, posFill, center);
    row.appendChild(bar);

    parent.appendChild(row);

    dashboard.meters.set(key, {
        row,
        value,
        fill: null,
        negFill,
        posFill,
        min,
        max,
        format: (v) => v.toFixed(2),
        lastSeen: 0,
        bidirectional: true,
    });
}

function registerVerticalMeter(parent, key, label, options) {
    const meter = document.createElement("div");
    meter.className = "power-meter";

    const head = document.createElement("div");
    head.className = "power-meter__head";

    const labelEl = document.createElement("span");
    labelEl.className = "power-meter__label";
    labelEl.textContent = label;

    const valueEl = document.createElement("span");
    valueEl.className = "power-meter__value";
    valueEl.textContent = options.placeholder || "--";

    head.append(labelEl, valueEl);

    const track = document.createElement("div");
    track.className = `power-meter__track${options.signed ? " power-meter__track--signed" : ""}`;

    let fill = null;
    let negFill = null;
    let posFill = null;

    if (options.signed) {
        negFill = document.createElement("div");
        negFill.className = "power-meter__fill power-meter__fill--neg";

        posFill = document.createElement("div");
        posFill.className = "power-meter__fill power-meter__fill--pos";

        const center = document.createElement("div");
        center.className = "power-meter__center";

        track.append(negFill, posFill, center);
    } else {
        fill = document.createElement("div");
        fill.className = "power-meter__fill";
        fill.style.setProperty("--value", "0");
        track.appendChild(fill);
    }

    meter.append(head, track);
    parent.appendChild(meter);

    dashboard.meters.set(key, {
        row: meter,
        value: valueEl,
        fill,
        negFill,
        posFill,
        min: options.min,
        max: options.max,
        format: options.format,
        lastSeen: 0,
        orientation: "vertical",
        bidirectional: !!options.signed,
    });
}

function buildThrusterPanel() {
    const panel = createPanel("Thrusters", "/thruster/thruster_i");
    const grid = document.createElement("div");
    grid.className = "compact-grid";
    panel.appendChild(grid);

    for (let i = 0; i < 8; i++) {
        registerMeter({
            parent: grid,
            key: `thruster_${i}`,
            label: `T${i}`,
            min: 1000,
            max: 2000,
            bar: true,
            placeholder: "1500 us",
            format: (v) => `${v.toFixed(0)} us`,
        });
    }
}

function buildPowerPanel() {
    const panel = createPanel("Power", "/power_monitor/monitor_i/*");
    const grid = document.createElement("div");
    grid.className = "power-grid";

    for (let i = 0; i < 8; i++) {
        const card = document.createElement("article");
        card.className = "power-card";

        const badge = document.createElement("div");
        badge.className = "power-card__badge";
        badge.textContent = `${i}`;

        const metrics = document.createElement("div");
        metrics.className = "power-card__metrics power-card__metrics--vertical";

        const spec = [
            { metric: "bus_voltage", label: "Bus", min: 0, max: 16, format: (v) => `${v.toFixed(2)} V`, color: "#57c7ff" },
            { metric: "current", label: "Current", min: 0, max: 20, format: (v) => `${v.toFixed(2)} A`, color: "#f0b35f" },
            { metric: "power", label: "Power", min: 0, max: 250, format: (v) => `${v.toFixed(1)} W`, color: "#73e0a8" },
        ];

        spec.forEach((item) => {
            registerVerticalMeter(metrics, `power_${i}_${item.metric}`, item.label, {
                min: item.min,
                max: item.max,
                format: item.format,
                placeholder: "--",
                signed: false,
            });

            const meter = dashboard.meters.get(`power_${i}_${item.metric}`);
            meter.row.classList.add("power-meter--accented");
            meter.row.style.setProperty("--meter-accent", item.color);
        });

        registerVerticalMeter(metrics, `power_${i}_shunt_voltage`, "Shunt", {
            min: -0.2,
            max: 0.2,
            format: (v) => `${v.toFixed(3)} V`,
            placeholder: "--",
            signed: true,
        });

        const shuntMeter = dashboard.meters.get(`power_${i}_shunt_voltage`);
        shuntMeter.row.classList.add("power-meter--accented", "power-meter--shunt");
        shuntMeter.row.style.setProperty("--meter-accent", "#f08d62");

        card.append(badge, metrics);
        grid.appendChild(card);
    }

    panel.appendChild(grid);
}

function buildArmPanel() {
    const panel = createPanel("Arm Motors", "/arm/arm_motor_i");
    const grid = document.createElement("div");
    grid.className = "compact-grid";
    panel.appendChild(grid);

    for (let i = 0; i < 6; i++) {
        registerMeter({
            parent: grid,
            key: `arm_motor_${i}`,
            label: `A${i}`,
            min: 1000,
            max: 2000,
            bar: true,
            placeholder: "1500 us",
            format: (v) => `${v.toFixed(0)} us`,
        });
    }
}

function buildDepthPanel() {
    const panel = createPanel("Depth", "/depth/*");
    const grid = document.createElement("div");
    grid.className = "depth-grid";

    registerMeter({
        parent: grid,
        key: "depth_depth",
        label: "Depth",
        min: 0,
        max: 100,
        bar: true,
        placeholder: "0.00 m",
        format: (v) => `${v.toFixed(2)} m`,
    });

    registerMeter({
        parent: grid,
        key: "depth_pressure",
        label: "Pressure",
        min: 0,
        max: 20,
        bar: true,
        placeholder: "0.00",
        format: (v) => `${v.toFixed(2)}`,
    });

    registerMeter({
        parent: grid,
        key: "depth_temperature",
        label: "Temp",
        min: 0,
        max: 40,
        bar: true,
        placeholder: "0.00 C",
        format: (v) => `${v.toFixed(2)} C`,
    });

    const sparkRow = document.createElement("div");
    sparkRow.className = "row";

    const sparkHead = document.createElement("div");
    sparkHead.className = "row__head";

    const sparkLabel = document.createElement("span");
    sparkLabel.className = "row__label";
    sparkLabel.textContent = "History";

    const sparkValue = document.createElement("span");
    sparkValue.className = "row__value";
    sparkValue.textContent = "--";

    sparkHead.append(sparkLabel, sparkValue);
    sparkRow.appendChild(sparkHead);

    const graph = document.createElement("canvas");
    graph.className = "depth-graph";

    sparkRow.appendChild(graph);
    grid.appendChild(sparkRow);

    dashboard.sparkline = {
        row: sparkRow,
        canvas: graph,
        value: sparkValue,
        lastSeen: 0,
    };

    panel.appendChild(grid);
}

function buildCommandsPanel() {
    const panel = createPanel("Commands", "/velocity_commands + /arm_commands");
    const wrap = document.createElement("div");
    wrap.className = "commands-wrap";

    const twistCol = document.createElement("div");
    twistCol.className = "commands-col";

    const twistTitle = document.createElement("div");
    twistTitle.className = "matrix__head";
    twistTitle.textContent = "Twist";
    twistCol.appendChild(twistTitle);

    registerSignedBarMeter(twistCol, "velocity_linear_x", "Lin X");
    registerSignedBarMeter(twistCol, "velocity_linear_y", "Lin Y");
    registerSignedBarMeter(twistCol, "velocity_linear_z", "Lin Z");
    registerSignedBarMeter(twistCol, "velocity_angular_x", "Ang X");
    registerSignedBarMeter(twistCol, "velocity_angular_y", "Ang Y");
    registerSignedBarMeter(twistCol, "velocity_angular_z", "Ang Z");

    const armCol = document.createElement("div");
    armCol.className = "commands-col";

    const armTitle = document.createElement("div");
    armTitle.className = "matrix__head";
    armTitle.textContent = "Arm Commands";
    armCol.appendChild(armTitle);

    for (let i = 0; i < 6; i++) {
        registerSignedBarMeter(armCol, `arm_commands_${i}`, `Arm ${i}`);
    }

    wrap.append(twistCol, armCol);
    panel.appendChild(wrap);
}

function buildCameraPanel() {
    const panel = createPanel("Cameras", "/video_0..3");
    const wrap = document.createElement("div");
    wrap.className = "camera-wrap";

    for (let cameraId = 0; cameraId < 4; cameraId++) {
        const card = document.createElement("div");
        card.className = "camera-card";

        const title = document.createElement("div");
        title.className = "camera-title";
        title.textContent = `CAM ${cameraId}`;

        const canvas = document.createElement("canvas");
        canvas.className = "camera-preview";
        canvas.width = 320;
        canvas.height = 240;

        card.append(title, canvas);
        wrap.appendChild(card);

        dashboard.cameras.set(cameraId, {
            card,
            canvas,
            lastSeen: 0,
        });
    }

    panel.appendChild(wrap);
}

function updateMeter(key, rawValue) {
    const meter = dashboard.meters.get(key);
    if (!meter || typeof rawValue !== "number" || Number.isNaN(rawValue)) {
        return;
    }

    const formatted = meter.format ? meter.format(rawValue) : String(rawValue);
    setText(meter.value, formatted);
    meter.lastSeen = Date.now();
    meter.row.classList.remove("is-stale");

    if (meter.fill) {
        const range = meter.max - meter.min || 1;
        const normalised = clamp((rawValue - meter.min) / range, 0, 1);
        if (meter.orientation === "vertical") {
            meter.fill.style.height = `${normalised * 100}%`;
        } else {
            meter.fill.style.setProperty("--value", String(normalised));
        }
    }

    if (meter.bidirectional) {
        const posMax = Math.max(meter.max, Number.EPSILON);
        const negMax = Math.max(Math.abs(meter.min), Number.EPSILON);
        const pos = clamp(rawValue > 0 ? rawValue / posMax : 0, 0, 1);
        const neg = clamp(rawValue < 0 ? Math.abs(rawValue) / negMax : 0, 0, 1);
        if (meter.orientation === "vertical") {
            meter.posFill.style.height = `${pos * 50}%`;
            meter.negFill.style.height = `${neg * 50}%`;
        } else {
            meter.posFill.style.width = `${pos * 50}%`;
            meter.negFill.style.width = `${neg * 50}%`;
        }
    }
}

function updateSparkline(key, values) {
    if (key !== "depth_array" || !dashboard.sparkline || !Array.isArray(values) || values.length === 0) {
        return;
    }

    const canvas = dashboard.sparkline.canvas;
    const context = canvas.getContext("2d");
    const fixedMin = 0;
    const fixedMax = 100;
    const size = resizeCanvasToDisplaySize(canvas, context);
    const w = size.width;
    const h = size.height;
    const leftPad = 32;
    const rightPad = 8;
    const topPad = 8;
    const bottomPad = 10;
    const graphW = w - leftPad - rightPad;
    const graphH = h - topPad - bottomPad;

    context.clearRect(0, 0, w, h);

    context.strokeStyle = "rgba(143, 176, 209, 0.18)";
    context.lineWidth = 1;
    context.fillStyle = "rgba(143, 176, 209, 0.4)";
    context.font = "10px IBM Plex Sans, Segoe UI, sans-serif";
    context.textAlign = "right";
    context.textBaseline = "middle";

    for (let i = 0; i <= 4; i++) {
        const y = topPad + (graphH / 4) * i;
        const depthVal = fixedMin + ((fixedMax - fixedMin) / 4) * i;
        context.beginPath();
        context.moveTo(leftPad, y);
        context.lineTo(w - rightPad, y);
        context.stroke();
        context.fillText(depthVal.toFixed(0), leftPad - 6, y);
    }

    context.strokeStyle = "#6fd9ff";
    context.lineWidth = 2;
    context.beginPath();

    values.forEach((sample, i) => {
        const ratio = values.length === 1 ? 0 : i / (values.length - 1);
        const x = leftPad + ratio * graphW;
        const normalised = clamp((sample - fixedMin) / (fixedMax - fixedMin), 0, 1);
        const y = topPad + normalised * graphH;

        if (i === 0) {
            context.moveTo(x, y);
        } else {
            context.lineTo(x, y);
        }
    });
    context.stroke();

    const latest = values[values.length - 1];
    const latestNorm = clamp((latest - fixedMin) / (fixedMax - fixedMin), 0, 1);
    const latestY = topPad + latestNorm * graphH;

    context.fillStyle = "#ff6b35";
    context.strokeStyle = "#ff8c5a";
    context.lineWidth = 2;
    context.beginPath();
    context.arc(w - rightPad - 4, latestY, 4, 0, Math.PI * 2);
    context.fill();
    context.stroke();

    context.fillStyle = "#ff6b35";
    context.font = "bold 10px IBM Plex Sans, Segoe UI, sans-serif";
    context.textAlign = "right";
    context.textBaseline = "bottom";
    context.fillText(`${latest.toFixed(1)}m`, w - rightPad - 10, latestY - 6);

    setText(dashboard.sparkline.value, `${latest.toFixed(2)} m`);
    dashboard.sparkline.lastSeen = Date.now();
    dashboard.sparkline.row.classList.remove("is-stale");
}

function updateCamera(cameraId, video) {
    const camera = dashboard.cameras.get(cameraId);
    if (!camera || !video || !Array.isArray(video.data)) {
        return;
    }

    const canvas = camera.canvas;
    const context = canvas.getContext("2d");
    const frame = context.createImageData(canvas.width, canvas.height);

    const sourceWidth = video.width || canvas.width;
    const sourceHeight = video.height || canvas.height;
    const source = video.data;

    for (let y = 0; y < canvas.height; y++) {
        for (let x = 0; x < canvas.width; x++) {
            const sx = Math.floor((x / canvas.width) * sourceWidth);
            const sy = Math.floor((y / canvas.height) * sourceHeight);
            const srcIndex = (sy * sourceWidth + sx) * 3;
            const dstIndex = (y * canvas.width + x) * 4;

            frame.data[dstIndex] = source[srcIndex] ?? 0;
            frame.data[dstIndex + 1] = source[srcIndex + 1] ?? 0;
            frame.data[dstIndex + 2] = source[srcIndex + 2] ?? 0;
            frame.data[dstIndex + 3] = 255;
        }
    }

    context.putImageData(frame, 0, 0);
    camera.lastSeen = Date.now();
    camera.card.classList.remove("is-stale");
}

function refreshStaleState() {
    const now = Date.now();
    const staleMs = 1600;

    dashboard.meters.forEach((meter) => {
        if (!meter.lastSeen || now - meter.lastSeen > staleMs) {
            meter.row.classList.add("is-stale");
        }
    });

    if (dashboard.sparkline && (!dashboard.sparkline.lastSeen || now - dashboard.sparkline.lastSeen > staleMs)) {
        dashboard.sparkline.row.classList.add("is-stale");
    }

    dashboard.cameras.forEach((camera) => {
        if (!camera.lastSeen || now - camera.lastSeen > staleMs) {
            camera.card.classList.add("is-stale");
        }
    });
}

function buildDashboard() {
    buildThrusterPanel();
    buildPowerPanel();
    buildArmPanel();
    buildDepthPanel();
    buildCommandsPanel();
    buildCameraPanel();
}

buildDashboard();
setInterval(refreshStaleState, 1000);
