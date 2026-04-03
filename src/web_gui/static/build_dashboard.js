
const dashboard = {
    meters: new Map(),
    sparkBars: new Map(),
    cameraCanvas: null,
    cameraDetails: null,
};

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

function formatValue(format, value) {
    if (typeof value === 'undefined' || value === null) {
        return '--';
    }

    return format ? format(value) : String(value);
}

function setStaleDisplay(entry, stale, currentValue, lastValue) {
    entry.card.classList.toggle('is-stale', stale);
    entry.value.textContent = stale ? 'No data' : currentValue;
    entry.secondary.textContent = stale ? `Last: ${lastValue}` : '';
}

function refreshStaleState() {
    const now = Date.now();

    dashboard.meters.forEach((entry) => {
        const stale = !entry.lastSeen || now - entry.lastSeen > 1000;
        const currentValue = formatValue(entry.format, entry.lastValue);
        setStaleDisplay(entry, stale, currentValue, currentValue);

        if (entry.kind === 'meter') {
            entry.fill.style.setProperty('--meter-value', stale ? '0' : `${entry.normalised ?? 0}`);
        }

        if (entry.kind === 'metric') {
            entry.fill.style.setProperty('--meter-value', stale ? '0' : `${entry.normalised ?? 0}`);
        }
    });

    dashboard.sparkBars.forEach((entry) => {
        const stale = !entry.lastSeen || now - entry.lastSeen > 1000;
        const currentValue = formatValue(entry.format, entry.lastValue);

        entry.card.classList.toggle('is-stale', stale);
        entry.value.textContent = stale ? 'No data' : currentValue;
        entry.secondary.textContent = stale ? `Last: ${currentValue}` : '';

        entry.bars.forEach((bar) => {
            bar.style.setProperty('--spark-value', stale ? '0' : `${bar.dataset.value ?? 0}`);
        });
    });

    if (dashboard.cameraCard) {
        const stale = !dashboard.cameraCard.lastSeen || now - dashboard.cameraCard.lastSeen > 1000;
        dashboard.cameraCard.card.classList.toggle('is-stale', stale);
        dashboard.cameraCard.details.textContent = stale
            ? `No data | Last: ${dashboard.cameraCard.lastInfo ?? 'unknown frame'}`
            : dashboard.cameraCard.lastInfo;
    }
}

function createSection(containerId, title, description) {
    const container = document.getElementById(containerId);
    const section = document.createElement('section');
    section.className = 'panel';

    const header = document.createElement('div');
    header.className = 'panel__header';

    const titleEl = document.createElement('h2');
    titleEl.textContent = title;

    const descriptionEl = document.createElement('p');
    descriptionEl.textContent = description;

    header.append(titleEl, descriptionEl);
    section.appendChild(header);

    const grid = document.createElement('div');
    grid.className = 'card-grid';
    section.appendChild(grid);
    container.appendChild(section);
    return grid;
}

function createGaugeCard(parent, options) {
    const card = document.createElement('article');
    card.className = 'card';

    const header = document.createElement('div');
    header.className = 'card__header';

    const title = document.createElement('h3');
    title.textContent = options.title;

    const topic = document.createElement('span');
    topic.className = 'card__topic';
    topic.textContent = options.topic;

    header.append(title, topic);

    const meter = document.createElement('div');
    meter.className = 'meter';

    const fill = document.createElement('div');
    fill.className = 'meter__fill';
    meter.appendChild(fill);

    const value = document.createElement('div');
    value.className = 'card__value';
    value.textContent = options.fallbackValue ?? 'Waiting for data';

    const secondary = document.createElement('div');
    secondary.className = 'card__secondary';
    secondary.textContent = '';

    card.append(header, meter, value, secondary);
    parent.appendChild(card);

    dashboard.meters.set(options.key, {
        kind: 'meter',
        card,
        fill,
        value,
        secondary,
        label: title,
        title: options.title,
        min: options.min,
        max: options.max,
        format: options.format,
        lastSeen: 0,
        lastValue: null,
        normalised: 0,
    });
}

function createMetricRow(parent, options) {
    const row = document.createElement('div');
    row.className = 'metric';

    const label = document.createElement('span');
    label.className = 'metric__label';
    label.textContent = options.label;

    const meter = document.createElement('div');
    meter.className = 'metric__meter';

    const fill = document.createElement('div');
    fill.className = 'metric__fill';
    meter.appendChild(fill);

    const value = document.createElement('span');
    value.className = 'metric__value';
    value.textContent = options.fallbackValue ?? '--';

    const secondary = document.createElement('span');
    secondary.className = 'metric__secondary';
    secondary.textContent = '';

    row.append(label, meter, value, secondary);
    parent.appendChild(row);

    dashboard.meters.set(options.key, {
        kind: 'metric',
        card: row,
        fill,
        value,
        secondary,
        label,
        title: options.label,
        min: options.min,
        max: options.max,
        format: options.format,
        lastSeen: 0,
        lastValue: null,
        normalised: 0,
    });
}

function createMultiMetricCard(parent, options) {
    const card = document.createElement('article');
    card.className = 'card card--stacked';

    const header = document.createElement('div');
    header.className = 'card__header';

    const title = document.createElement('h3');
    title.textContent = options.title;

    const topic = document.createElement('span');
    topic.className = 'card__topic';
    topic.textContent = options.topic;

    header.append(title, topic);

    const body = document.createElement('div');
    body.className = 'card__metrics';

    options.metrics.forEach((metric) => {
        createMetricRow(body, metric);
    });

    card.append(header, body);
    parent.appendChild(card);
}

function createSparklineCard(parent, options) {
    const card = document.createElement('article');
    card.className = 'card';

    const header = document.createElement('div');
    header.className = 'card__header';

    const title = document.createElement('h3');
    title.textContent = options.title;

    const topic = document.createElement('span');
    topic.className = 'card__topic';
    topic.textContent = options.topic;

    header.append(title, topic);

    const sparkline = document.createElement('div');
    sparkline.className = 'sparkline';

    const bars = [];
    for (let index = 0; index < options.length; index++) {
        const bar = document.createElement('div');
        bar.className = 'sparkline__bar';
        sparkline.appendChild(bar);
        bars.push(bar);
    }

    const value = document.createElement('div');
    value.className = 'card__value';
    value.textContent = options.fallbackValue ?? 'Waiting for data';

    const secondary = document.createElement('div');
    secondary.className = 'card__secondary';
    secondary.textContent = '';

    card.append(header, sparkline, value, secondary);
    parent.appendChild(card);

    dashboard.sparkBars.set(options.key, {
        card,
        bars,
        value,
        secondary,
        format: options.format,
        lastSeen: 0,
        lastValue: null,
        values: [],
        lastInfo: '',
    });
}

function createCameraCard(parent, options) {
    const card = document.createElement('article');
    card.className = 'card card--camera';

    const header = document.createElement('div');
    header.className = 'card__header';

    const title = document.createElement('h3');
    title.textContent = options.title;

    const topic = document.createElement('span');
    topic.className = 'card__topic';
    topic.textContent = options.topic;

    header.append(title, topic);

    const preview = document.createElement('canvas');
    preview.width = 80;
    preview.height = 60;
    preview.className = 'camera-preview';

    const details = document.createElement('div');
    details.className = 'camera-details';
    details.textContent = 'Waiting for video frame';

    card.append(header, preview, details);
    parent.appendChild(card);

    dashboard.cameraCanvas = preview;
    dashboard.cameraDetails = details;
    dashboard.cameraCard = {
        card,
        preview,
        details,
        lastSeen: 0,
        lastInfo: 'Waiting for video frame',
    };
}

function updateMeter(key, value) {
    const meter = dashboard.meters.get(key);
    if (!meter) {
        return;
    }

    const normalised = meter.max === meter.min ? 0 : clamp((value - meter.min) / (meter.max - meter.min), 0, 1);
    meter.fill.style.setProperty('--meter-value', `${normalised}`);
    meter.normalised = normalised;
    meter.lastSeen = Date.now();
    meter.lastValue = value;
    meter.card.classList.remove('is-stale');
    meter.value.textContent = formatValue(meter.format, value);
    meter.secondary.textContent = '';
}

function updateSparkline(key, values) {
    const sparkline = dashboard.sparkBars.get(key);
    if (!sparkline) {
        return;
    }

    if (!Array.isArray(values) || values.length === 0) {
        sparkline.value.textContent = '--';
        sparkline.bars.forEach((bar) => {
            bar.style.setProperty('--spark-value', '0');
            bar.dataset.value = '0';
        });
        return;
    }

    const min = Math.min(...values);
    const max = Math.max(...values);
    const range = max - min || 1;

    sparkline.bars.forEach((bar, index) => {
        const nextValue = values[index] ?? values[values.length - 1];
        const normalised = clamp((nextValue - min) / range, 0, 1);
        bar.style.setProperty('--spark-value', `${normalised}`);
        bar.dataset.value = `${normalised}`;
    });

    sparkline.lastSeen = Date.now();
    sparkline.lastValue = values[values.length - 1];
    sparkline.values = values;
    sparkline.card.classList.remove('is-stale');
    sparkline.value.textContent = formatValue(sparkline.format, values[values.length - 1]);
    sparkline.secondary.textContent = '';
}

function updateCamera(video) {
    if (!dashboard.cameraCanvas || !video) {
        return;
    }

    const canvas = dashboard.cameraCanvas;
    const context = canvas.getContext('2d');
    const imageData = context.createImageData(canvas.width, canvas.height);
    const sourceWidth = video.width || canvas.width;
    const sourceHeight = video.height || canvas.height;
    const source = video.data || [];

    for (let y = 0; y < canvas.height; y++) {
        for (let x = 0; x < canvas.width; x++) {
            const sourceX = Math.floor((x / canvas.width) * sourceWidth);
            const sourceY = Math.floor((y / canvas.height) * sourceHeight);
            const sourceIndex = (sourceY * sourceWidth + sourceX) * 3;
            const targetIndex = (y * canvas.width + x) * 4;

            imageData.data[targetIndex] = source[sourceIndex] ?? 0;
            imageData.data[targetIndex + 1] = source[sourceIndex + 1] ?? 0;
            imageData.data[targetIndex + 2] = source[sourceIndex + 2] ?? 0;
            imageData.data[targetIndex + 3] = 255;
        }
    }

    context.putImageData(imageData, 0, 0);
    dashboard.cameraCard.lastSeen = Date.now();
    dashboard.cameraCard.lastInfo = `${video.width} x ${video.height} | ${video.encoding} | ${source.length} bytes`;
    dashboard.cameraCard.card.classList.remove('is-stale');
    dashboard.cameraCard.details.textContent = dashboard.cameraCard.lastInfo;
}

function buildDashboard() {
    const thrusterGrid = createSection('dashboard', 'Thrusters', 'PWM output from each thruster channel');
    const powerGrid = createSection('dashboard', 'Power Monitor', 'Each thruster has a linked power monitor');
    const armGrid = createSection('dashboard', 'Arm', 'PWM output for each arm motor');
    const depthGrid = createSection('dashboard', 'Depth', 'Depth sensor and derived readings');
    const commandGrid = createSection('dashboard', 'Commands', 'Vehicle velocity and arm command inputs');
    const cameraGrid = createSection('dashboard', 'Camera', 'Live image stream preview');

    for (let thrusterId = 0; thrusterId < 8; thrusterId++) {
        createGaugeCard(thrusterGrid, {
            key: `thruster_${thrusterId}`,
            title: `Thruster ${thrusterId}`,
            topic: `/thruster/thruster_${thrusterId}`,
            min: 1000,
            max: 2000,
            fallbackValue: 'PWM: 1500 us',
            format: (value) => `PWM: ${value.toFixed(1)} us`,
        });
    }

    for (let thrusterId = 0; thrusterId < 8; thrusterId++) {
        createMultiMetricCard(powerGrid, {
            title: `Thruster ${thrusterId} monitor`,
            topic: `/power_monitor/monitor_${thrusterId}`,
            metrics: [
                {
                    key: `power_${thrusterId}_bus_voltage`,
                    label: 'Bus voltage',
                    min: 0,
                    max: 16,
                    fallbackValue: '12.0 V',
                    format: (value) => `${value.toFixed(2)} V`,
                },
                {
                    key: `power_${thrusterId}_current`,
                    label: 'Current',
                    min: 0,
                    max: 20,
                    fallbackValue: '0.0 A',
                    format: (value) => `${value.toFixed(2)} A`,
                },
                {
                    key: `power_${thrusterId}_power`,
                    label: 'Power',
                    min: 0,
                    max: 250,
                    fallbackValue: '0.0 W',
                    format: (value) => `${value.toFixed(2)} W`,
                },
                {
                    key: `power_${thrusterId}_shunt_voltage`,
                    label: 'Shunt',
                    min: -0.2,
                    max: 0.2,
                    fallbackValue: '0.000 V',
                    format: (value) => `${value.toFixed(3)} V`,
                },
            ],
        });
    }

    for (let armMotorId = 0; armMotorId < 6; armMotorId++) {
        createGaugeCard(armGrid, {
            key: `arm_motor_${armMotorId}`,
            title: `Arm ${armMotorId}`,
            topic: `/arm/arm_motor_${armMotorId}`,
            min: 1000,
            max: 2000,
            fallbackValue: 'PWM: 1500 us',
            format: (value) => `PWM: ${value.toFixed(1)} us`,
        });
    }

    createGaugeCard(depthGrid, {
        key: 'depth_depth',
        title: 'Depth',
        topic: '/depth/depth',
        min: 0,
        max: 100,
        fallbackValue: '0.0 m',
        format: (value) => `${value.toFixed(2)} m`,
    });

    createGaugeCard(depthGrid, {
        key: 'depth_pressure',
        title: 'Pressure',
        topic: '/depth/pressure',
        min: 0,
        max: 20,
        fallbackValue: '1.0 bar',
        format: (value) => `${value.toFixed(2)} bar`,
    });

    createGaugeCard(depthGrid, {
        key: 'depth_temperature',
        title: 'Temperature',
        topic: '/depth/temperature',
        min: 0,
        max: 40,
        fallbackValue: '20.0 C',
        format: (value) => `${value.toFixed(2)} C`,
    });

    createSparklineCard(depthGrid, {
        key: 'depth_array',
        title: 'Depth array',
        topic: '/depth/depth_array',
        length: 10,
        fallbackValue: 'Waiting for samples',
        format: (value) => `${value.toFixed(2)} m`,
    });

    createMultiMetricCard(commandGrid, {
        title: 'Velocity commands',
        topic: '/velocity_commands',
        metrics: [
            { key: 'velocity_linear_x', label: 'Linear x', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
            { key: 'velocity_linear_y', label: 'Linear y', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
            { key: 'velocity_linear_z', label: 'Linear z', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
            { key: 'velocity_angular_x', label: 'Angular x', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
            { key: 'velocity_angular_y', label: 'Angular y', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
            { key: 'velocity_angular_z', label: 'Angular z', min: -0.5, max: 0.5, fallbackValue: '0.00', format: (value) => value.toFixed(2) },
        ],
    });

    const armCommandsCard = document.createElement('article');
    armCommandsCard.className = 'card card--stacked';

    const armCommandsHeader = document.createElement('div');
    armCommandsHeader.className = 'card__header';

    const armCommandsTitle = document.createElement('h3');
    armCommandsTitle.textContent = 'Arm commands';

    const armCommandsTopic = document.createElement('span');
    armCommandsTopic.className = 'card__topic';
    armCommandsTopic.textContent = '/arm_commands';

    armCommandsHeader.append(armCommandsTitle, armCommandsTopic);

    const armCommandsBody = document.createElement('div');
    armCommandsBody.className = 'card__metrics';

    for (let armId = 0; armId < 6; armId++) {
        createMetricRow(armCommandsBody, {
            key: `arm_commands_${armId}`,
            label: `Arm ${armId}`,
            min: -0.5,
            max: 0.5,
            fallbackValue: '0.00',
            format: (value) => value.toFixed(2),
        });
    }

    armCommandsCard.append(armCommandsHeader, armCommandsBody);
    commandGrid.appendChild(armCommandsCard);

    createCameraCard(cameraGrid, {
        title: 'Video stream',
        topic: '/video_0',
    });
}

buildDashboard();

setInterval(refreshStaleState, 1000);