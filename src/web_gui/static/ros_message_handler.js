const pendingRosMessages = new Map();
let rosFlushScheduled = false;

function processRosMessage(topic, data) {
    if (topic.startsWith('/thruster/')) {
        const parts = topic.split('/');
        const thrusterName = parts[2];

        if (thrusterName) {
            if (parts[3] === 'response_pwm') {
                updateThrusterMeter(thrusterName, 'response', data);
            } else {
                updateThrusterMeter(thrusterName, 'target', data);
            }
        }
        return;
    }

    if (topic.startsWith('/power_monitor/')) {
        const [monitorTopic, metric] = topic.split('/').slice(2);
        const monitorId = monitorTopic.split('_')[1];
        updateMeter(`power_${monitorId}_${metric}`, data);
        return;
    }

    if (topic.startsWith('/arm/')) {
        const parts = topic.split('/');
        const armMotorName = parts[2];

        if (armMotorName) {
            if (parts[3] === 'response_pwm') {
                updateArmMeter(armMotorName, 'response', data);
            } else {
                updateArmMeter(armMotorName, 'target', data);
            }
        }
        return;
    }

    if (topic === '/depth/depth') {
        updateMeter('depth_depth', data);
        return;
    }

    if (topic === '/depth/pressure') {
        updateMeter('depth_pressure', data / 100000); // Convert Pa to bar
        return;
    }

    if (topic === '/depth/temperature') {
        updateMeter('depth_temperature', data);
        return;
    }

    if (topic === '/depth/depth_array') {
        updateSparkline('depth_array', data);
        return;
    }

    if (topic === '/velocity_commands') {
        updateMeter('velocity_linear_x', data.linear.x);
        updateMeter('velocity_linear_y', data.linear.y);
        updateMeter('velocity_linear_z', data.linear.z);
        updateMeter('velocity_angular_x', data.angular.x);
        updateMeter('velocity_angular_y', data.angular.y);
        updateMeter('velocity_angular_z', data.angular.z);
        return;
    }

    if (topic === '/arm_commands') {
        data.forEach((value, index) => {
            updateMeter(`arm_commands_${index}`, value);
        });
        return;
    }

    if ((topic === '/imu' || topic === 'imu' || topic.startsWith('/imu/')) && typeof updateModelOrientation === 'function') {
        if (typeof window.updateImuTelemetry === 'function') {
            window.updateImuTelemetry(data);
        }
        
        if (data && typeof data === 'object' && data.orientation) {
            updateModelOrientation(data);
        } else if (Array.isArray(data) && data.length >= 3) {
            updateModelOrientation(data[0], data[1], data[2]);
        } else if (data && typeof data === 'object') {
            updateModelOrientation(data);
        }
        return;
    }

    if (topic.startsWith('/camera_')) {
        const cameraId = Number(topic.split('_')[1].split('/')[0]);
        if (!Number.isNaN(cameraId)) {
            updateCamera(cameraId, data);
        }
    }
}

function flushRosMessages() {
    rosFlushScheduled = false;

    if (pendingRosMessages.size === 0) {
        return;
    }

    const messages = Array.from(pendingRosMessages.entries());
    pendingRosMessages.clear();

    for (const [topic, data] of messages) {
        processRosMessage(topic, data);
    }

    if (pendingRosMessages.size > 0 && !rosFlushScheduled) {
        rosFlushScheduled = true;
        window.requestAnimationFrame(flushRosMessages);
    }
}

const pendingFrames = new Map();
let frameFlushScheduled = false;

function flushCameraFrames() {
    frameFlushScheduled = false;
    const frames = Array.from(pendingFrames.entries());
    pendingFrames.clear();
    for (const [cameraId, bytes] of frames) {
        updateCamera(cameraId, bytes);
    }
}

ws.binaryType = 'arraybuffer';

ws.onmessage = (event) => {
    if (typeof event.data !== 'string') {
        const buf = new Uint8Array(event.data);
        const cameraId = buf[0];
        pendingFrames.set(cameraId, buf.subarray(1));
        if (!frameFlushScheduled) {
            frameFlushScheduled = true;
            window.requestAnimationFrame(flushCameraFrames);
        }
        return;
    }

    if (event.data === 'connected') {
        console.log('WebSocket initialized!');
        return;
    }

    const { topic, data } = JSON.parse(event.data);
    pendingRosMessages.set(topic, data);

    if (!rosFlushScheduled) {
        rosFlushScheduled = true;
        window.requestAnimationFrame(flushRosMessages);
    }
};