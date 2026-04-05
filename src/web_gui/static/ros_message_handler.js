ws.onmessage = (event) => {
    const { topic, data } = JSON.parse(event.data);

    if (topic.startsWith('/thruster/')) {
        const thrusterId = topic.split('/')[2].split('_')[1];
        updateMeter(`thruster_${thrusterId}`, data);
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
        updateMeter('depth_pressure', data);
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

    if (topic.startsWith('/camera_')) {
        const cameraId = Number(topic.split('_')[1].split('/')[0]);
        if (!Number.isNaN(cameraId)) {
            updateCamera(cameraId, data);
        }
    }
};