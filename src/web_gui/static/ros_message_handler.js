const pendingRosMessages = new Map();
let rosFlushScheduled = false;

function processRosMessage(topic, data) {
    if (topic.startsWith('/thruster/')) {
        const parts = topic.split('/');
        const thrusterId = parts[2]?.split('_')[1];
        if (!Number.isNaN(Number(thrusterId))) {
            if (parts[3] === 'response_pwm') {
                updateThrusterMeter(`thruster_${thrusterId}`, 'response', data);
            } else {
                updateThrusterMeter(`thruster_${thrusterId}`, 'target', data);
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
        const armMotorId = topic.split('/')[2].split('_')[2];
        updateMeter(`arm_motor_${armMotorId}`, data);
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

ws.onmessage = (event) => {
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