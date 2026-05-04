class MessageTester {
    constructor() {
        this.onmessage = null;
        this.publishRate = 100;
        this.depthHistory = [];
        this.depthHistoryLength = 60;
        this.startTime = Date.now();

        this.start();
    }

    eulerToQuaternion(roll, pitch, yaw) {
        const halfRoll = roll * 0.5;
        const halfPitch = pitch * 0.5;
        const halfYaw = yaw * 0.5;

        const cr = Math.cos(halfRoll);
        const sr = Math.sin(halfRoll);
        const cp = Math.cos(halfPitch);
        const sp = Math.sin(halfPitch);
        const cy = Math.cos(halfYaw);
        const sy = Math.sin(halfYaw);

        return {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        };
    }

    createImuMessage() {
        const elapsed = (Date.now() - this.startTime) / 1000;
        const nowMillis = Date.now();
        const sec = Math.floor(nowMillis / 1000);
        const nanosec = (nowMillis % 1000) * 1e6;

        const roll = Math.sin(elapsed * 0.45) * 0.45;
        const pitch = Math.cos(elapsed * 0.35) * 0.35;
        const yaw = elapsed * 0.25;
        const quaternion = this.eulerToQuaternion(roll, pitch, yaw);

        return {
            header: {
                stamp: {
                    sec,
                    nanosec,
                },
                frame_id: 'IMU_Frame',
            },
            angular_velocity: {
                x: Math.cos(elapsed * 0.45) * 0.18,
                y: -Math.sin(elapsed * 0.35) * 0.12,
                z: 0.25,
            },
            linear_acceleration: {
                x: Math.sin(elapsed * 0.9) * 0.55,
                y: Math.cos(elapsed * 0.8) * 0.45,
                z: 9.81 + Math.sin(elapsed * 0.6) * 0.18,
            },
            linear_velocity: {
                x: Math.sin(elapsed * 0.9) * 0.55,
                y: Math.cos(elapsed * 0.8) * 0.45,
                z: 9.81 + Math.sin(elapsed * 0.6) * 0.18,
            },
            orientation: quaternion,
        };
    }

    generateDepthSample() {
        const elapsed = (Date.now() - this.startTime) / 1000;
        const baseline = 48 + Math.sin(elapsed / 8) * 18;
        const wave = Math.sin(elapsed * 1.7) * 2.5 + Math.cos(elapsed * 0.9) * 1.2;
        return Math.max(0, Math.min(5, (baseline + wave) / 20));
    }

    recordDepth(depth) {
        this.depthHistory.push(depth);
        if (this.depthHistory.length > this.depthHistoryLength) {
            this.depthHistory.shift();
        }
    }

    start() {
        window.testInterval = setInterval(() => {
            // Thruster PWM + power monitor data (x8)
            for (let thrusterId = 0; thrusterId < 8; thrusterId++) {
                const pwm = Math.sin(Date.now() / 1000 + thrusterId) * 500 + 1500;
                this.sendMessage({ topic: `/thruster/thruster_${thrusterId}`, data: parseFloat(pwm) });


                const power_monitor_topic = `/power_monitor/monitor_${thrusterId}`;

                const bus_voltage = 12 + Math.cos(Date.now() / 1000 + thrusterId) * 0.5;
                this.sendMessage({ topic: power_monitor_topic + "/bus_voltage", data: parseFloat(bus_voltage) });

                const current = Math.abs(Math.sin(Date.now() / 1000 + thrusterId)) * 10;
                this.sendMessage({ topic: power_monitor_topic + "/current", data: parseFloat(current) });

                const power = bus_voltage * current;
                this.sendMessage({ topic: power_monitor_topic + "/power", data: parseFloat(power) });

                const shunt_voltage = Math.cos(Date.now() / 1000 + thrusterId) * 0.1;
                this.sendMessage({ topic: power_monitor_topic + "/shunt_voltage", data: parseFloat(shunt_voltage) });
            }

            
            // Arm PWM data (x6)
            for (let armMotorId = 0; armMotorId < 6; armMotorId++) {
                const pwm = Math.cos(Date.now() / 1000 + armMotorId) * 500 + 1500;
                this.sendMessage({ topic: `/arm/arm_motor_${armMotorId}`, data: parseFloat(pwm) });
            }

            // Depth sensor data
            const depth = this.generateDepthSample();
            this.recordDepth(depth);
            this.sendMessage({ topic: '/depth/depth', data: parseFloat(depth) });

            this.sendMessage({ topic: '/depth/depth_array', data: this.depthHistory.map((sample) => parseFloat(sample.toFixed(2))) });
            
            const pressure = (1 + depth * 0.1 + Math.cos(Date.now() / 5000) * 0.5) * 100000; // pressure in pascals (simulated)
            this.sendMessage({ topic: '/depth/pressure', data: parseFloat(pressure) });

            const temperature = 20 + Math.cos(Date.now() / 5000) * 5;
            this.sendMessage({ topic: '/depth/temperature', data: parseFloat(temperature) });

            // IMU message data (sensor_msgs/Imu shape)
            this.sendMessage({ topic: '/imu', data: this.createImuMessage() });

            // Camera streaming data (CompressedImage with JPEG format)
            const testCanvas = document.createElement('canvas');
            testCanvas.width = 640;
            testCanvas.height = 480;
            const testCtx = testCanvas.getContext('2d');

            // Draw a test pattern
            for (let y = 0; y < 480; y += 40) {
                for (let x = 0; x < 640; x += 40) {
                    testCtx.fillStyle = `hsl(${(x + y + Math.round(Date.now() / 10)) % 360}, 40%, 50%)`;
                    testCtx.fillRect(x, y, 40, 40);
                }
            }

            // Encode as JPEG and extract raw bytes
            const dataUrl = testCanvas.toDataURL('image/jpeg', 0.8);
            const base64 = dataUrl.split(',')[1];
            const binary = atob(base64);
            const jpegBytes = Array.from(binary, c => c.charCodeAt(0));

            const video = {
                header: {
                    stamp: Date.now(),
                    frame_id: 'camera_0'
                },
                format: 'jpeg',
                data: jpegBytes
            };
            this.sendMessage({ topic: '/camera_0/image/compressed', data: video });
            this.sendMessage({ topic: '/camera_1/image/compressed', data: video });
            this.sendMessage({ topic: '/camera_2/image/compressed', data: video });
            this.sendMessage({ topic: '/camera_3/image/compressed', data: video });

            /* ===== Houston data ===== */
            // Twist data type (controls thrusters)
            const velocity_commands = {
                linear: {
                    x: Math.sin(Date.now() / 1000),
                    y: Math.cos(Date.now() / 1000),
                    z: Math.sin(Date.now() / 2000)
                },
                angular: {
                    x: Math.cos(Date.now() / 1000),
                    y: Math.sin(Date.now() / 1000),
                    z: Math.cos(Date.now() / 2000)
                }
            };
            this.sendMessage({ topic: '/velocity_commands', data: velocity_commands });

            // Float32MultiArray data type (controls arm)
            const arm_commands = Array.from({ length: 6 }, (_, i) => Math.sin(Date.now() / 1000 + i));
            this.sendMessage({ topic: '/arm_commands', data: arm_commands.map(c => parseFloat(c)) });
        }, this.publishRate);
    }

    sendMessage(message) {
        // console.log('Sent message:', message);
        if (this.onmessage) {
            this.onmessage({ data: JSON.stringify(message) });
        }
    }
}

const ws = new MessageTester();