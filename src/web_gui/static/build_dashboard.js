/**
 * publishes button values to /gui_buttons/
 *  - scan_crabs: toggles crab scanning mode
 *  - photogrammetry: toggles photogrammetry mode
 *  - measure_iceberg: triggers iceberg measurement sequence
 */


const dashboard = {
    meters: new Map(),
    thrusterMeters: new Map(),
    armMeters: new Map(),
    sparkline: null,
    cameras: new Map(),
    thrusterViz: null,
    model3d: null,
};

function publishMessage(topic, data) {
    if (typeof window.ws !== 'undefined' && window.ws && window.ws.send) {
        const message = JSON.stringify({ topic, data });
        window.ws.send(message);
    }
}

function downloadAllCameraScreenshots() {
    const timestamp = Date.now();
    for (let cameraId = 0; cameraId < 4; cameraId++) {
        const camera = dashboard.cameras.get(cameraId);
        if (!camera || !camera.canvas) {
            console.warn(`Camera ${cameraId} not available for screenshot`);
            continue;
        }

        const canvas = camera.canvas;
        canvas.toBlob((blob) => {
            if (!blob) {
                console.error(`Failed to create blob from camera ${cameraId}`);
                return;
            }

            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = `camera${cameraId}_${timestamp}.png`;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);
        }, 'image/png');
    }
}

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

function resizeRendererToDisplaySize(renderer) {
    const canvas = renderer.domElement;
    const cssWidth = Math.max(1, Math.floor(canvas.clientWidth || 1));
    const cssHeight = Math.max(1, Math.floor(canvas.clientHeight || 1));
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const pixelWidth = Math.max(1, Math.floor(cssWidth * dpr));
    const pixelHeight = Math.max(1, Math.floor(cssHeight * dpr));

    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        renderer.setPixelRatio(dpr);
        renderer.setSize(cssWidth, cssHeight, false);
        return true;
    }

    return false;
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

function registerDualMeter(parent, key, label, collection) {
    const card = document.createElement("div");
    card.className = "thruster-meter"; // reusing the CSS classes since they fit

    const head = document.createElement("div");
    head.className = "row__head";

    const name = document.createElement("span");
    name.className = "row__label";
    name.textContent = label;

    const targetDisplay = document.createElement("span");
    targetDisplay.className = "thruster-meter__target-display";
    targetDisplay.textContent = "TARGET --";

    const value = document.createElement("span");
    value.className = "row__value";
    value.textContent = "RESP --";

    head.append(name, targetDisplay, value);

    const track = document.createElement("div");
    track.className = "thruster-meter__track";

    const responseFill = document.createElement("div");
    responseFill.className = "thruster-meter__response";

    const targetMarker = document.createElement("div");
    targetMarker.className = "thruster-meter__target";

    track.append(responseFill, targetMarker);

    card.append(head, track);
    parent.appendChild(card);

    collection.set(key, {
        card,
        value,
        responseFill,
        targetMarker,
        targetDisplay,
        min: -1.0,
        max: 1.0,
        lastSeen: 0.0,
    });
}

function registerThrusterMeter(parent, key, label) {
    registerDualMeter(parent, key, label, dashboard.thrusterMeters);
}

function registerArmMeter(parent, key, label) {
    registerDualMeter(parent, key, label, dashboard.armMeters);
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

function registerSignedBarMeter(parent, key, label, min = -1, max = 1) {
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
    panel.id = "panel-thrusters";
    const grid = document.createElement("div");
    grid.className = "compact-grid";
    panel.appendChild(grid);

    const thrusters = [
        { key: "thruster_fll", label: "Front Left Lateral" },
        { key: "thruster_frl", label: "Front Right Lateral" },
        { key: "thruster_rll", label: "Rear Left Lateral" },
        { key: "thruster_rrl", label: "Rear Right Lateral" },
        { key: "thruster_flv", label: "Front Left Vertical" },
        { key: "thruster_frv", label: "Front Right Vertical" },
        { key: "thruster_rlv", label: "Rear Left Vertical" },
        { key: "thruster_rrv", label: "Rear Right Vertical" },
    ];

    thrusters.forEach((thruster) => {
        registerThrusterMeter(
            grid,
            thruster.key,
            thruster.label
        );
    });

    const vizCard = document.createElement("div");
    vizCard.className = "thruster-viz-card";

    const vizGrid = document.createElement("div");
    vizGrid.className = "thruster-viz-grid";

    const xPanel = document.createElement("div");
    xPanel.className = "thruster-viz-panel";
    const xHead = document.createElement("div");
    xHead.className = "row__head";
    const xLabel = document.createElement("span");
    xLabel.className = "row__label";
    xLabel.textContent = "X-Drive";
    xHead.appendChild(xLabel);
    const xCanvas = document.createElement("canvas");
    xCanvas.className = "thruster-viz";
    xPanel.append(xHead, xCanvas);

    const upPanel = document.createElement("div");
    upPanel.className = "thruster-viz-panel";
    const upHead = document.createElement("div");
    upHead.className = "row__head";
    const upLabel = document.createElement("span");
    upLabel.className = "row__label";
    upLabel.textContent = "Up Motors";
    upHead.appendChild(upLabel);
    const upCanvas = document.createElement("canvas");
    upCanvas.className = "thruster-viz";
    upPanel.append(upHead, upCanvas);

    vizGrid.append(xPanel, upPanel);
    vizCard.append(vizGrid);
    panel.appendChild(vizCard);

    dashboard.thrusterViz = {
        card: vizCard,
        xCanvas,
        upCanvas,
        xValues: [1500, 1500, 1500, 1500],
        upValues: [1500, 1500, 1500, 1500],
    };

    drawThrusterViz();
    drawUpMotorViz();
}

function updateDualMeter(meter, kind, rawValue, isThruster = false, key = "") {
    if (!meter || typeof rawValue !== "number" || Number.isNaN(rawValue)) {
        return;
    }

    const range = meter.max - meter.min || 1;
    const normalised = clamp((rawValue - meter.min) / range, 0, 1);

    meter.lastSeen = Date.now();
    meter.card.classList.remove("is-stale");

    if (kind === "response") {
        meter.responseFill.style.width = `${normalised * 100}%`;
        setText(meter.value, `RESP ${rawValue.toFixed(2)} us`);
        
        // Update visualization for response values (which represent actual thrust)
        if (isThruster && dashboard.thrusterViz) {
            const lateralMap = {
                thruster_fll: 0,
                thruster_frl: 1,
                thruster_rll: 2,
                thruster_rrl: 3,
            };

            const verticalMap = {
                thruster_flv: 0,
                thruster_frv: 1,
                thruster_rlv: 2,
                thruster_rrv: 3,
            };

            if (key in lateralMap) {
                dashboard.thrusterViz.xValues[lateralMap[key]] = rawValue;
                drawThrusterViz();
            }
            else if (key in verticalMap) {
                dashboard.thrusterViz.upValues[verticalMap[key]] = rawValue;
                drawUpMotorViz();
            }
        }
        return;
    }

    if (kind === "target") {
        meter.targetMarker.style.left = `${normalised * 100}%`;
        setText(meter.targetDisplay, `TARGET ${rawValue.toFixed(2)} us`);
    }
}

function updateThrusterMeter(key, kind, rawValue) {
    updateDualMeter(dashboard.thrusterMeters.get(key), kind, rawValue, true, key);
}

function updateArmMeter(key, kind, rawValue) {
    updateDualMeter(dashboard.armMeters.get(key), kind, rawValue, false, key);
}

function drawThrusterArrow(context, x, y, dx, dy, color) {
    const len = Math.hypot(dx, dy);
    if (len < 1) {
        return;
    }

    context.strokeStyle = color;
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(x + dx, y + dy);
    context.stroke();

    const ux = dx / len;
    const uy = dy / len;
    const head = clamp(len * 0.3, 4, 10);
    const wing = clamp(head * 0.45, 2, 5);

    context.beginPath();
    context.moveTo(x + dx, y + dy);
    context.lineTo(x + dx - ux * head - uy * wing, y + dy - uy * head + ux * wing);
    context.lineTo(x + dx - ux * head + uy * wing, y + dy - uy * head - ux * wing);
    context.closePath();
    context.fillStyle = color;
    context.fill();
}

function drawThrusterViz() {
    const viz = dashboard.thrusterViz;
    if (!viz) {
        return;
    }

    const context = viz.xCanvas.getContext("2d");
    const size = resizeCanvasToDisplaySize(viz.xCanvas, context);
    const w = size.width;
    const h = size.height;

    context.clearRect(0, 0, w, h);

    const cx = w * 0.5;
    const cy = h * 0.55;
    const robotSize = Math.min(w, h) * 0.42;
    const half = robotSize * 0.5;

    // Top-down robot body (square) with a front indicator.
    context.fillStyle = "rgba(16, 34, 56, 0.92)";
    context.strokeStyle = "rgba(143, 176, 209, 0.5)";
    context.lineWidth = 1.5;
    context.fillRect(cx - half, cy - half, robotSize, robotSize);
    context.strokeRect(cx - half, cy - half, robotSize, robotSize);

    context.beginPath();
    context.moveTo(cx - robotSize * 0.12, cy - half);
    context.lineTo(cx + robotSize * 0.12, cy - half);
    context.lineTo(cx, cy - half - 9);
    context.closePath();
    context.fillStyle = "rgba(124, 217, 255, 0.85)";
    context.fill();

    const corners = [
        { x: cx - half, y: cy - half }, // top-left
        { x: cx + half, y: cy - half }, // top-right
        { x: cx - half, y: cy + half }, // bottom-left
        { x: cx + half, y: cy + half }, // bottom-right
    ];

    // X-drive directions in robot frame (y forward/up):
    // TL and BR: +x +y
    // TR and BL: -x +y
    const dirs = [
        { x: 1, y: 1 },
        { x: -1, y: 1 },
        { x: -1, y: 1 },
        { x: 1, y: 1 },
    ];

    const norm = 1 / Math.sqrt(2);
    const maxLen = Math.min(w, h) * 0.2;

    for (let i = 0; i < 4; i++) {
        const pwm = viz.xValues[i] ?? 1500;
        const force = clamp((pwm - 1500) / 500, -1, 1);
        const mag = Math.abs(force);

        const ux = dirs[i].x * norm;
        const uy = dirs[i].y * norm;
        const sign = force >= 0 ? 1 : -1;
        const dx = ux * maxLen * mag * sign;
        const dy = -uy * maxLen * mag * sign;

        const pos = corners[i];

        context.beginPath();
        context.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
        context.fillStyle = "rgba(231, 242, 255, 0.9)";
        context.fill();

        drawThrusterArrow(context, pos.x, pos.y, dx, dy, "#57c7ff");

        context.fillStyle = "rgba(143, 176, 209, 0.85)";
        context.font = "10px IBM Plex Sans, Segoe UI, sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText(`T${i}`, pos.x, pos.y + 12);
    }
}

function drawUpMotorViz() {
    const viz = dashboard.thrusterViz;
    if (!viz) {
        return;
    }

    const context = viz.upCanvas.getContext("2d");
    const size = resizeCanvasToDisplaySize(viz.upCanvas, context);
    const w = size.width;
    const h = size.height;

    context.clearRect(0, 0, w, h);

    const cx = w * 0.5;
    const cy = h * 0.55;
    const robotSize = Math.min(w, h) * 0.42;
    const half = robotSize * 0.5;

    context.fillStyle = "rgba(16, 34, 56, 0.92)";
    context.strokeStyle = "rgba(143, 176, 209, 0.5)";
    context.lineWidth = 1.5;
    context.fillRect(cx - half, cy - half, robotSize, robotSize);
    context.strokeRect(cx - half, cy - half, robotSize, robotSize);

    const corners = [
        { x: cx - half, y: cy - half },
        { x: cx + half, y: cy - half },
        { x: cx - half, y: cy + half },
        { x: cx + half, y: cy + half },
    ];

    const maxLen = Math.min(w, h) * 0.18;
    for (let i = 0; i < 4; i++) {
        const pwm = viz.upValues[i] ?? 1500;
        const force = clamp((pwm - 1500) / 500, -1, 1);
        const dy = -maxLen * force;
        const pos = corners[i];

        context.beginPath();
        context.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
        context.fillStyle = "rgba(231, 242, 255, 0.9)";
        context.fill();

        drawThrusterArrow(context, pos.x, pos.y, 0, dy, "#73e0a8");

        context.fillStyle = "rgba(143, 176, 209, 0.85)";
        context.font = "10px IBM Plex Sans, Segoe UI, sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText(`T${i + 4}`, pos.x, pos.y + 12);
    }
}

function buildPowerPanel() {
    const panel = createPanel("Power", "/power_monitor/monitor_i/*");
    panel.id = "panel-power";
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
            // { metric: "power", label: "Power", min: 0, max: 250, format: (v) => `${v.toFixed(1)} W`, color: "#73e0a8" },
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

        // registerVerticalMeter(metrics, `power_${i}_shunt_voltage`, "Shunt", {
        //     min: -0.2,
        //     max: 0.2,
        //     format: (v) => `${v.toFixed(3)} V`,
        //     placeholder: "--",
        //     signed: true,
        // });

        // const shuntMeter = dashboard.meters.get(`power_${i}_shunt_voltage`);
        // shuntMeter.row.classList.add("power-meter--accented", "power-meter--shunt");
        // shuntMeter.row.style.setProperty("--meter-accent", "#f08d62");

        card.append(badge, metrics);
        grid.appendChild(card);
    }

    panel.appendChild(grid);
}

function buildArmPanel() {
    const panel = createPanel("", "");
    panel.id = "panel-arm";
    const layout = document.createElement("div");
    layout.className = "arm-layout";
    panel.appendChild(layout);

    const buttonsSection = document.createElement("div");
    buttonsSection.className = "arm-section arm-section--buttons";

    const buttonsTitle = document.createElement("div");
    buttonsTitle.className = "arm-section__title";
    buttonsTitle.textContent = "Buttons";
    buttonsSection.appendChild(buttonsTitle);

    const buttonGrid = document.createElement("div");
    buttonGrid.className = "test-button-grid";

    const buttonConfigs = [
        { label: "Scan Crabs", toggle: true, topic: "/gui_buttons/scan_crabs" },
        { label: "Take Screenshot", toggle: false, action: "screenshot" },
        { label: "Photogrammetry", toggle: true, topic: "/gui_buttons/photogrammetry" },
        { label: "Measure Iceberg", toggle: false, action: "measure_iceberg" },
        { label: "Quad Cam", toggle: true, action: "toggle_quad_cam" },
    ];

    for (const config of buttonConfigs) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "test-button-grid__button";
        button.textContent = config.label;

        if (config.toggle && config.topic) {
            button.classList.add("test-button-grid__button--toggle");
            button.setAttribute("aria-pressed", "false");
            button.addEventListener("click", () => {
                const isPressed = button.classList.toggle("is-active");
                button.setAttribute("aria-pressed", String(isPressed));
                publishMessage(config.topic, isPressed);
            });
        } else if (config.action === "screenshot") {
            button.addEventListener("click", downloadAllCameraScreenshots);
        } else if (config.action === "measure_iceberg") {
            button.addEventListener("click", () => {
                publishMessage("/gui_buttons/measure_iceberg", "pressed");
            });
        } else if (config.action === "toggle_quad_cam") {
            button.classList.add("test-button-grid__button--toggle");
            button.setAttribute("aria-pressed", "false");
            button.addEventListener("click", () => {
                const isPressed = button.classList.toggle("is-active");
                button.setAttribute("aria-pressed", String(isPressed));
                const dashboardEl = document.getElementById("dashboard");
                if (isPressed) {
                    dashboardEl.classList.add("quad-mode");
                } else {
                    dashboardEl.classList.remove("quad-mode");
                }
            });
        }

        buttonGrid.appendChild(button);
    }

    buttonsSection.appendChild(buttonGrid);

    const motorsSection = document.createElement("div");
    motorsSection.className = "arm-section arm-section--motors";

    const motorsTitle = document.createElement("div");
    motorsTitle.className = "arm-section__title";
    motorsTitle.textContent = "Arm Motors";
    motorsSection.appendChild(motorsTitle);

    const motorsGrid = document.createElement("div");
    motorsGrid.className = "compact-grid arm-section__grid";
    motorsSection.appendChild(motorsGrid);

    layout.append(buttonsSection, motorsSection);

    const armMotors = [
        { key: "motor_base_yaw", label: "Base Yaw" },
        { key: "motor_base_pitch", label: "Base Pitch" },
        { key: "motor_elbow_pitch", label: "Elbow Pitch" },
        { key: "motor_wrist_yaw", label: "Wrist Yaw" },
        { key: "motor_wrist_pitch", label: "Wrist Pitch" },
        { key: "motor_claw", label: "Claw" },
    ];

    armMotors.forEach((motor) => {
        registerArmMeter(
            motorsGrid,
            motor.key,
            motor.label
        );
    });
}

function buildDepthPanel() {
    const panel = createPanel("Depth", "/depth/*");
    panel.id = "panel-depth";
    const grid = document.createElement("div");
    grid.className = "depth-grid";

    registerMeter({
        parent: grid,
        key: "depth_depth",
        label: "Depth",
        min: 0,
        max: 5,
        bar: true,
        placeholder: "0.00 m",
        format: (v) => `${v.toFixed(2)} m`,
    });

    registerMeter({
        parent: grid,
        key: "depth_pressure",
        label: "Pressure",
        min: 0,
        max: 5,
        bar: true,
        placeholder: "0.00 bar",
        format: (v) => `${v.toFixed(2)} bar`,
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
    sparkRow.className = "row depth-history-row";

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

function buildCameraPanel() {
    const panel = createPanel("Cameras", "/camera_0..3/image/compressed");
    panel.id = "panel-cameras";
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

        card.style.cursor = "pointer";
        card.addEventListener("click", () => {
            const dashboardEl = document.getElementById("dashboard");
            if (dashboardEl.classList.contains("cinema-mode")) {
                dashboardEl.classList.remove("cinema-mode");
                card.classList.remove("camera-card--cinema");
            } else {
                dashboardEl.classList.add("cinema-mode");
                card.classList.add("camera-card--cinema");
            }
        });

        dashboard.cameras.set(cameraId, {
            card,
            canvas,
            lastSeen: 0,
        });
    }

    panel.appendChild(wrap);
}

function normalizeLoadedModel(THREE, root) {
    const bounds = new THREE.Box3().setFromObject(root);
    const size = bounds.getSize(new THREE.Vector3());
    const center = bounds.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z, Number.EPSILON);
    const targetSize = 1.8;
    const scale = targetSize / maxDim;

    root.position.sub(center);
    root.scale.setScalar(scale);
    root.position.y += 0.15;
}

function loadRovModel(THREE, modelPath = "/static/robot.glb") {
    const GLTFLoader = window.THREE_GLTFLoader;
    if (!GLTFLoader) {
        return Promise.reject(new Error("GLTFLoader not available"));
    }

    const loader = new GLTFLoader();
    return new Promise((resolve, reject) => {
        loader.load(
            modelPath,
            (gltf) => {
                const modelRoot = gltf.scene || gltf.scenes?.[0];
                if (!modelRoot) {
                    reject(new Error("robot.glb has no scene"));
                    return;
                }

                modelRoot.traverse((node) => {
                    if (node.isMesh) {
                        node.castShadow = false;
                        node.receiveShadow = false;
                    }
                });

                normalizeLoadedModel(THREE, modelRoot);
                resolve(modelRoot);
            },
            undefined,
            (error) => reject(error)
        );
    });
}

function applyModelOrientation() {
    if (!dashboard.model3d) {
        return;
    }

    const { roll, pitch, yaw } = dashboard.model3d.orientation;
    if (dashboard.model3d.model) {
        // FLU IMU body frame -> Three.js model frame
        // Model authored with nose along +Z; signs verified in testing
        dashboard.model3d.model.rotation.set(pitch, 0, roll, "XYZ");
    }

    if (dashboard.model3d.grid) {
        dashboard.model3d.grid.rotation.y = -yaw;
    }

    if (dashboard.model3d.navballCanvas) {
        drawNavball(dashboard.model3d.navballCanvas, pitch, roll);
    }

    const toDeg = (radians) => radians * (180 / Math.PI);
    const displayPitch = -pitch;  // show pilot-friendly sign on the HUD
    setText(
        dashboard.model3d.readout,
        `ROLL ${toDeg(roll).toFixed(1)} deg | PITCH ${toDeg(displayPitch).toFixed(1)} deg | YAW ${toDeg(yaw).toFixed(1)} deg`
    );
}

function formatImuComponent(value, digits = 3) {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(digits) : "--";
}

function updateImuTelemetry(imuPayload) {
    if (!dashboard.model3d || !dashboard.model3d.telemetry || !imuPayload || typeof imuPayload !== "object") {
        return;
    }

    const telemetry = dashboard.model3d.telemetry;
    const orientation = imuPayload.orientation && typeof imuPayload.orientation === "object"
        ? imuPayload.orientation
        : imuPayload;
    const angularVelocity = imuPayload.angular_velocity && typeof imuPayload.angular_velocity === "object"
        ? imuPayload.angular_velocity
        : imuPayload;
    const linearVelocity = imuPayload.linear_velocity && typeof imuPayload.linear_velocity === "object"
        ? imuPayload.linear_velocity
        : imuPayload.linear_acceleration && typeof imuPayload.linear_acceleration === "object"
            ? imuPayload.linear_acceleration
        : imuPayload;

    const gx = angularVelocity.x ?? angularVelocity.gyro_x;
    const gy = angularVelocity.y ?? angularVelocity.gyro_y;
    const gz = angularVelocity.z ?? angularVelocity.gyro_z;
    setText(
        telemetry.gyro,
        `x ${formatImuComponent(gx)} y ${formatImuComponent(gy)} z ${formatImuComponent(gz)}`
    );

    const vx = linearVelocity.x ?? linearVelocity.vel_x ?? linearVelocity.accel_x;
    const vy = linearVelocity.y ?? linearVelocity.vel_y ?? linearVelocity.accel_y;
    const vz = linearVelocity.z ?? linearVelocity.vel_z ?? linearVelocity.accel_z;
    setText(
        telemetry.linearVelocity,
        `x ${formatImuComponent(vx)} y ${formatImuComponent(vy)} z ${formatImuComponent(vz)}`
    );

    dashboard.model3d.lastSeen = Date.now();
    dashboard.model3d.rows.forEach((row) => row.classList.remove("is-stale"));
}

function quaternionToEulerXYZ(qx, qy, qz, qw) {
    const x = Number(qx);
    const y = Number(qy);
    const z = Number(qz);
    const w = Number(qw);

    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(w)) {
        return null;
    }

    const norm = Math.hypot(x, y, z, w);
    if (norm <= Number.EPSILON) {
        return null;
    }

    const nx = x / norm;
    const ny = y / norm;
    const nz = z / norm;
    const nw = w / norm;

    const sinrCosp = 2 * (nw * nx + ny * nz);
    const cosrCosp = 1 - 2 * (nx * nx + ny * ny);
    const roll = Math.atan2(sinrCosp, cosrCosp);

    const sinp = 2 * (nw * ny - nz * nx);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

    const sinyCosp = 2 * (nw * nz + nx * ny);
    const cosyCosp = 1 - 2 * (ny * ny + nz * nz);
    const yaw = Math.atan2(sinyCosp, cosyCosp);

    return { roll, pitch, yaw };
}

function updateModelOrientation(rollOrOrientation, pitch, yaw, unit = "rad") {
    let rollValue = 0;
    let pitchValue = 0;
    let yawValue = 0;
    let unitValue = unit;

    if (typeof rollOrOrientation === "object" && rollOrOrientation !== null) {
        updateImuTelemetry(rollOrOrientation);
    }

    if (typeof rollOrOrientation === "object" && rollOrOrientation !== null) {
        if (rollOrOrientation.orientation && typeof rollOrOrientation.orientation === "object") {
            const euler = quaternionToEulerXYZ(
                rollOrOrientation.orientation.x,
                rollOrOrientation.orientation.y,
                rollOrOrientation.orientation.z,
                rollOrOrientation.orientation.w
            );
            if (!euler) {
                return;
            }
            rollValue = euler.roll;
            pitchValue = euler.pitch;
            yawValue = euler.yaw;
            unitValue = "rad";
        } else {
            const hasQuaternion =
                Number.isFinite(Number(rollOrOrientation.w ?? rollOrOrientation.real)) &&
                Number.isFinite(Number(rollOrOrientation.x ?? rollOrOrientation.i)) &&
                Number.isFinite(Number(rollOrOrientation.y ?? rollOrOrientation.j)) &&
                Number.isFinite(Number(rollOrOrientation.z ?? rollOrOrientation.k));

            if (hasQuaternion) {
                const euler = quaternionToEulerXYZ(
                    rollOrOrientation.x ?? rollOrOrientation.i,
                    rollOrOrientation.y ?? rollOrOrientation.j,
                    rollOrOrientation.z ?? rollOrOrientation.k,
                    rollOrOrientation.w ?? rollOrOrientation.real
                );
                if (!euler) {
                    return;
                }
                rollValue = euler.roll;
                pitchValue = euler.pitch;
                yawValue = euler.yaw;
                unitValue = "rad";
            } else {
                rollValue = Number(rollOrOrientation.roll ?? rollOrOrientation.x ?? 0);
                pitchValue = Number(rollOrOrientation.pitch ?? rollOrOrientation.y ?? 0);
                yawValue = Number(rollOrOrientation.yaw ?? rollOrOrientation.z ?? 0);
                unitValue = rollOrOrientation.unit || unitValue;
            }
        }
    } else {
        rollValue = Number(rollOrOrientation ?? 0);
        pitchValue = Number(pitch ?? 0);
        yawValue = Number(yaw ?? 0);
    }

    if (!Number.isFinite(rollValue) || !Number.isFinite(pitchValue) || !Number.isFinite(yawValue)) {
        return;
    }

    if (unitValue === "deg" || unitValue === "degree" || unitValue === "degrees") {
        const scale = Math.PI / 180;
        rollValue *= scale;
        pitchValue *= scale;
        yawValue *= scale;
    }

    if (!dashboard.model3d) {
        return;
    }

    dashboard.model3d.orientation = {
        roll: rollValue,
        pitch: pitchValue,
        yaw: yawValue,
    };

    applyModelOrientation();
}

function drawNavball(canvas, pitchRad, rollRad) {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const { width: w, height: h } = resizeCanvasToDisplaySize(canvas, ctx);
    const cx = w / 2;
    const cy = h / 2;
    const radius = Math.min(w, h) / 2 * 0.85;

    ctx.clearRect(0, 0, w, h);

    // Save and clip to the instrument circle
    ctx.save();
    ctx.translate(cx, cy);
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.clip();

    // Roll: Horizon rotates opposite to camera/vehicle roll
    ctx.rotate(-rollRad);

    // Pitch: maps +/- 90 degrees to a scaled radius
    const maxPitchDeg = 90;
    const pitchScale = 3;
    const pitchDeg = pitchRad * (180 / Math.PI);
    const pitchOffset = -(pitchDeg / maxPitchDeg) * radius * pitchScale;

    // Sky
    ctx.fillStyle = "#3498db";
    ctx.fillRect(-radius * 10, -radius * 10 + pitchOffset, radius * 20, radius * 10);

    // Ground
    ctx.fillStyle = "#8b4513";
    ctx.fillRect(-radius * 10, pitchOffset, radius * 20, radius * 10);

    // Horizon line
    ctx.strokeStyle = "white";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-radius * 10, pitchOffset);
    ctx.lineTo(radius * 10, pitchOffset);
    ctx.stroke();

    // Pitch lines
    ctx.fillStyle = "white";
    ctx.strokeStyle = "white";
    ctx.font = "10px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let p = -80; p <= 80; p += 10) {
        if (p === 0) continue;
        const lineOffset = pitchOffset - (p / maxPitchDeg) * radius * pitchScale;
        if (Math.abs(lineOffset) > radius * 0.95) continue; // skip if off ball

        const isMajor = p % 30 === 0;
        const width = isMajor ? radius * 0.6 : radius * 0.3;

        ctx.beginPath();
        ctx.moveTo(-width / 2, lineOffset);
        ctx.lineTo(width / 2, lineOffset);
        ctx.stroke();

        ctx.fillText(p.toString(), -width / 2 - 12, lineOffset);
        ctx.fillText(p.toString(), width / 2 + 12, lineOffset);
    }

    ctx.restore();

    // Fixed ROV reference
    ctx.strokeStyle = "yellow";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx - radius * 0.4, cy);
    ctx.lineTo(cx - radius * 0.1, cy);
    ctx.lineTo(cx, cy + radius * 0.1);
    ctx.lineTo(cx + radius * 0.1, cy);
    ctx.lineTo(cx + radius * 0.4, cy);
    ctx.stroke();

    // Little dot in center
    ctx.fillStyle = "yellow";
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fill();

    // Outer ring
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.lineWidth = 3;
    ctx.strokeStyle = "#2f5d80";
    ctx.stroke();
}

function buildModelPanel() {
    const panel = createPanel("IMU", "/imu");
    panel.id = "panel-model";
    const wrap = document.createElement("div");
    wrap.className = "model3d-wrap";

    const canvasesWrap = document.createElement("div");
    canvasesWrap.className = "model3d-canvases";
    canvasesWrap.style.position = "relative";
    canvasesWrap.style.display = "flex";
    canvasesWrap.style.flexDirection = "column";

    const canvas = document.createElement("canvas");
    canvas.className = "model3d-canvas";
    canvas.style.flex = "1";
    canvas.style.minHeight = "180px";

    const navballCanvas = document.createElement("canvas");
    navballCanvas.className = "navball-canvas";
    navballCanvas.style.position = "absolute";
    navballCanvas.style.top = "6px";
    navballCanvas.style.right = "6px";
    navballCanvas.style.width = "120px";
    navballCanvas.style.height = "120px";
    navballCanvas.style.borderRadius = "50%";
    navballCanvas.style.border = "2px solid rgba(47, 93, 128, 0.8)";
    navballCanvas.style.background = "#06101c";
    navballCanvas.style.zIndex = "10";

    canvasesWrap.append(canvas, navballCanvas);

    const readout = document.createElement("div");
    readout.className = "model3d-readout row";

    const readoutHead = document.createElement("div");
    readoutHead.className = "row__head";

    const readoutLabel = document.createElement("span");
    readoutLabel.className = "row__label";
    readoutLabel.textContent = "Orientation";

    const readoutValue = document.createElement("span");
    readoutValue.className = "row__value";
    readoutValue.textContent = "ROLL 0.0 deg | PITCH 0.0 deg | YAW 0.0 deg";

    readoutHead.append(readoutLabel, readoutValue);
    readout.appendChild(readoutHead);

    const details = document.createElement("div");
    details.className = "model3d-details";

    const createTelemetryRow = (labelText, initialText = "--") => {
        const row = document.createElement("div");
        row.className = "row model3d-row";

        const head = document.createElement("div");
        head.className = "row__head";

        const label = document.createElement("span");
        label.className = "row__label";
        label.textContent = labelText;

        const value = document.createElement("span");
        value.className = "row__value";
        value.textContent = initialText;

        head.append(label, value);
        row.appendChild(head);
        details.appendChild(row);

        return { row, value };
    };

    const gyroRow = createTelemetryRow("Angular Vel (rad/s)", "x -- y -- z --");
    const linearVelocityRow = createTelemetryRow("Linear Vel", "x -- y -- z --");

    wrap.append(canvasesWrap, readout, details);
    panel.appendChild(wrap);

    dashboard.model3d = {
        canvas,
        navballCanvas,
        readout: readoutValue,
        rows: [readout, gyroRow.row, linearVelocityRow.row],
        telemetry: {
            gyro: gyroRow.value,
            linearVelocity: linearVelocityRow.value,
        },
        orientation: {
            roll: 0,
            pitch: 0,
            yaw: 0,
        },
        lastSeen: 0,
        renderer: null,
        camera: null,
        scene: null,
        model: null,
        grid: null,
        animationFrame: 0,
    };

    const THREE = window.THREE;
    if (!THREE || !window.THREE_GLTFLoader) {
        const context = canvas.getContext("2d");
        const size = resizeCanvasToDisplaySize(canvas, context);
        context.clearRect(0, 0, size.width, size.height);
        context.fillStyle = "rgba(231, 242, 255, 0.9)";
        context.font = "12px IBM Plex Sans, Segoe UI, sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText("three.js or GLTFLoader unavailable", size.width * 0.5, size.height * 0.5);
        return;
    }

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    renderer.setClearColor(0x000000, 0);

    const scene = new THREE.Scene();
    scene.add(new THREE.AmbientLight(0x9dbfdc, 5.95));

    const keyLight = new THREE.DirectionalLight(0xcce7ff, 12.05);
    keyLight.position.set(2.6, 2.2, 1.4);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x7fc2ff, 2.45);
    fillLight.position.set(-2.2, 1.2, -1.8);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(5, 12, 0x2f5d80, 0x183652);
    grid.position.y = -0.45;
    scene.add(grid);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
    camera.position.set(0, 1.8, -2.7);
    camera.lookAt(0, 0, 0);

    dashboard.model3d.renderer = renderer;
    dashboard.model3d.scene = scene;
    dashboard.model3d.camera = camera;
    dashboard.model3d.grid = grid;
    dashboard.model3d.model = null;

    loadRovModel(THREE)
        .then((model) => {
            dashboard.model3d.model = model;
            scene.add(model);
            applyModelOrientation();
        })
        .catch((error) => {
            setText(dashboard.model3d.readout, "Failed to load /static/robot.glb");
            console.error("Failed to load /static/robot.glb", error);
        });

    const renderLoop = () => {
        if (!dashboard.model3d || !dashboard.model3d.renderer || !dashboard.model3d.camera) {
            return;
        }

        const resized = resizeRendererToDisplaySize(dashboard.model3d.renderer);
        if (resized) {
            dashboard.model3d.camera.aspect = dashboard.model3d.canvas.clientWidth / dashboard.model3d.canvas.clientHeight;
            dashboard.model3d.camera.updateProjectionMatrix();
        }

        dashboard.model3d.renderer.render(dashboard.model3d.scene, dashboard.model3d.camera);
        dashboard.model3d.animationFrame = window.requestAnimationFrame(renderLoop);
    };

    applyModelOrientation();
    renderLoop();
}

window.updateModelOrientation = updateModelOrientation;
window.updateThrusterMeter = updateThrusterMeter;
window.updateImuTelemetry = updateImuTelemetry;

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
    const latest = values[values.length - 1];

    if (canvas.clientWidth === 0 || canvas.clientHeight === 0) {
        setText(dashboard.sparkline.value, `${latest.toFixed(2)} m`);
        dashboard.sparkline.lastSeen = Date.now();
        dashboard.sparkline.row.classList.remove("is-stale");
        return; // Skip rendering and resizing if canvas is hidden
    }

    const context = canvas.getContext("2d");
    const fixedMin = 0;
    const fixedMax = 5;
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

    const blob = new Blob(
        [new Uint8Array(video.data)],
        { type: `image/${video.format || 'jpeg'}` }
    );
    const url = URL.createObjectURL(blob);
    const img = new Image();

    img.onload = () => {
        context.drawImage(img, 0, 0, canvas.width, canvas.height);
        URL.revokeObjectURL(url);
        camera.lastSeen = Date.now();
        camera.card.classList.remove("is-stale");
    };

    img.onerror = () => {
        URL.revokeObjectURL(url);
        console.error(`Failed to load image for camera ${cameraId}`);
    };

    img.src = url;
}

function refreshStaleState() {
    const now = Date.now();
    const staleMs = 1600;

    dashboard.meters.forEach((meter) => {
        if (!meter.lastSeen || now - meter.lastSeen > staleMs) {
            meter.row.classList.add("is-stale");
        } else {
            meter.row.classList.remove("is-stale");
        }
    });

    dashboard.thrusterMeters.forEach((meter) => {
        if (!meter.lastSeen || now - meter.lastSeen > staleMs) {
            meter.card.classList.add("is-stale");
        } else {
            meter.card.classList.remove("is-stale");
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

    if (dashboard.model3d && Array.isArray(dashboard.model3d.rows)) {
        const isStale = !dashboard.model3d.lastSeen || now - dashboard.model3d.lastSeen > staleMs;
        dashboard.model3d.rows.forEach((row) => {
            if (isStale) {
                row.classList.add("is-stale");
            } else {
                row.classList.remove("is-stale");
            }
        });
    }
}

function buildDashboard() {
    buildThrusterPanel();
    buildPowerPanel();
    buildArmPanel();
    buildDepthPanel();
    buildCameraPanel();
    buildModelPanel();
}

buildDashboard();
setInterval(refreshStaleState, 1000);

document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
        const dashboardEl = document.getElementById("dashboard");
        if (dashboardEl && dashboardEl.classList.contains("cinema-mode")) {
            dashboardEl.classList.remove("cinema-mode");
            document.querySelectorAll(".camera-card--cinema").forEach(card => {
                card.classList.remove("camera-card--cinema");
            });
        }
        else if (dashboardEl && dashboardEl.classList.contains("quad-mode")) {
            dashboardEl.classList.remove("quad-mode");
            // Find the untoggle button and update its state
            const quadButton = Array.from(document.querySelectorAll(".test-button-grid__button")).find(btn => btn.textContent === "Quad Cam");
            if (quadButton && quadButton.classList.contains("is-active")) {
                quadButton.classList.remove("is-active");
                quadButton.setAttribute("aria-pressed", "false");
            }
        }
    }
    if (e.key === "q" || e.key === "Q") {
        const dashboardEl = document.getElementById("dashboard");
        if (dashboardEl) {
            dashboardEl.classList.toggle("quad-mode");
            // Find the toggle button and update its state
            const quadButton = Array.from(document.querySelectorAll(".test-button-grid__button")).find(btn => btn.textContent === "Quad Cam");
            if (quadButton) {
                const isActive = dashboardEl.classList.contains("quad-mode");
                if (isActive) {
                    quadButton.classList.add("is-active");
                } else {
                    quadButton.classList.remove("is-active");
                }
                quadButton.setAttribute("aria-pressed", String(isActive));
            }
        }
    }
});

