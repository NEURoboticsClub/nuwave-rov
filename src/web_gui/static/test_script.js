class MessageTester {
    constructor() {
        this.onmessage = null;
        this.publishRate = 100;
        this.depthHistory = [];
        this.depthHistoryLength = 60;
        this.startTime = Date.now();

        this.start();
    }

    generateDepthSample() {
        const elapsed = (Date.now() - this.startTime) / 1000;
        const baseline = 48 + Math.sin(elapsed / 8) * 18;
        const wave = Math.sin(elapsed * 1.7) * 2.5 + Math.cos(elapsed * 0.9) * 1.2;
        return Math.max(0, Math.min(100, baseline + wave));
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
            
            const pressure = 1 + depth * 0.1 + Math.cos(Date.now() / 5000) * 0.5;
            this.sendMessage({ topic: '/depth/pressure', data: parseFloat(pressure) });

            const temperature = 20 + Math.cos(Date.now() / 5000) * 5;
            this.sendMessage({ topic: '/depth/temperature', data: parseFloat(temperature) });

            // Camera streaming data (using cv2_to_imgmsg)
            const video = {
                header: {
                    stamp: Date.now(),
                    frame_id: 'camera_0'
                },
                height: 480,
                width: 640,
                encoding: 'bgr8',
                is_bigendian: 0,
                step: 640 * 3,
                data: Array.from({ length: 480 * 640 * 3 }, () => Math.floor(Math.random() * 256))
            };
            this.sendMessage({ topic: '/video_0', data: video });

            /* ===== Houston data ===== */
            // Twist data type (controls thrusters)
            const velocity_commands = {
                linear: {
                    x: Math.sin(Date.now() / 1000) * 0.5,
                    y: Math.cos(Date.now() / 1000) * 0.5,
                    z: Math.sin(Date.now() / 2000) * 0.5
                },
                angular: {
                    x: Math.cos(Date.now() / 1000) * 0.5,
                    y: Math.sin(Date.now() / 1000) * 0.5,
                    z: Math.cos(Date.now() / 2000) * 0.5
                }
            };
            this.sendMessage({ topic: '/velocity_commands', data: velocity_commands });

            // Float32MultiArray data type (controls arm)
            const arm_commands = Array.from({ length: 6 }, (_, i) => Math.sin(Date.now() / 1000 + i) * 0.5);
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