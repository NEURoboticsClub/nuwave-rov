const dashboard = {
    meters: new Map(),
    sparkline: null,
    cameras: new Map(),
    thrusterViz: null,
    model3d: null,
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
    const panel = createPanel("Cameras", "/camera_0..3/image/compressed");
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

function loadRovModel(THREE, modelPath = "robot.glb") {
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
        // GLB local axes are rotated relative to IMU axes.
        // Map IMU {roll, pitch, yaw} -> model {x, y, z} as {pitch, yaw, roll}.
        dashboard.model3d.model.rotation.set(pitch, yaw, roll, "XYZ");
    }

    const toDeg = (radians) => radians * (180 / Math.PI);
    setText(
        dashboard.model3d.readout,
        `ROLL ${toDeg(roll).toFixed(1)} deg | PITCH ${toDeg(pitch).toFixed(1)} deg | YAW ${toDeg(yaw).toFixed(1)} deg`
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

function buildModelPanel() {
    const panel = createPanel("IMU", "/imu");
    const wrap = document.createElement("div");
    wrap.className = "model3d-wrap";

    const canvas = document.createElement("canvas");
    canvas.className = "model3d-canvas";

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

    wrap.append(canvas, readout, details);
    panel.appendChild(wrap);

    dashboard.model3d = {
        canvas,
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
    scene.add(new THREE.AmbientLight(0x9dbfdc, 0.55));

    const keyLight = new THREE.DirectionalLight(0xcce7ff, 1.05);
    keyLight.position.set(2.6, 2.2, 1.4);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x7fc2ff, 0.45);
    fillLight.position.set(-2.2, 1.2, -1.8);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(5, 12, 0x2f5d80, 0x183652);
    grid.position.y = -0.45;
    scene.add(grid);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
    camera.position.set(2.4, 1.8, 2.7);
    camera.lookAt(0, 0, 0);

    dashboard.model3d.renderer = renderer;
    dashboard.model3d.scene = scene;
    dashboard.model3d.camera = camera;
    dashboard.model3d.model = null;

    loadRovModel(THREE, "robot.glb")
        .then((model) => {
            dashboard.model3d.model = model;
            scene.add(model);
            applyModelOrientation();
        })
        .catch((error) => {
            setText(dashboard.model3d.readout, "Failed to load robot.glb");
            console.error("Failed to load robot.glb", error);
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

function updateMeter(key, rawValue) {
    const meter = dashboard.meters.get(key);
    if (!meter || typeof rawValue !== "number" || Number.isNaN(rawValue)) {
        return;
    }

    const formatted = meter.format ? meter.format(rawValue) : String(rawValue);
    setText(meter.value, formatted);
    meter.lastSeen = Date.now();
    meter.row.classList.remove("is-stale");

    if (key.startsWith("thruster_") && dashboard.thrusterViz) {
        const thrusterId = Number(key.split("_")[1]);
        if (!Number.isNaN(thrusterId) && thrusterId >= 0 && thrusterId < 4) {
            dashboard.thrusterViz.xValues[thrusterId] = rawValue;
            drawThrusterViz();
        } else if (!Number.isNaN(thrusterId) && thrusterId >= 4 && thrusterId < 8) {
            dashboard.thrusterViz.upValues[thrusterId - 4] = rawValue;
            drawUpMotorViz();
        }
    }

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
    // buildCommandsPanel();
    buildCameraPanel();
    buildModelPanel();
}

buildDashboard();
setInterval(refreshStaleState, 1000);
