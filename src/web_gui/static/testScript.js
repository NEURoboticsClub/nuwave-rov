class MessageTester {
    constructor() {
        this.onmessage = null;
        this.start();
    }
    start() {
        setInterval(() => {
            for (let thrusterId = 0; thrusterId < 8; thrusterId++) {
                const pwm = Math.sin(Date.now() / 1000 + thrusterId) * 500 + 1500;
                this.sendMessage({ topic: `/thruster/thruster_${thrusterId}`, data: parseFloat(pwm) });


                const power_monitor_topic = `/power_monitor/monitor_${thrusterId}`;

                const bus_voltage = 12 + Math.cos(Date.now() / 1000 + thrusterId) * 0.5;
                this.sendMessage({ topic: power_monitor_topic + "/bus_voltage", data: parseFloat(bus_voltage) });

                const current = Math.abs(Math.sin(Date.now() / 1000 + thrusterId)) * 10;
                this.sendMessage({ topic: power_monitor_topic + "/current", data: parseFloat(current) });

                

            }

            for (let armMotorId = 0; armMotorId < 6; armMotorId++) {
                const pwm = Math.cos(Date.now() / 1000 + armMotorId) * 500 + 1500;
                this.sendMessage({ topic: `/arm/arm_motor_${armMotorId}`, data: parseFloat(pwm) });
            }

            
        }, 1000);
    }

    sendMessage(message) {
        // console.log('Sent message:', message);
        if (this.onmessage) {
            this.onmessage({ data: JSON.stringify(message) });
        }
    }
}
const testws = new MessageTester();

testws.onmessage = (event) => {
    const { topic, data } = JSON.parse(event.data);

    if (topic.startsWith('/thruster/')) {
        const thrusterId = topic.split('/')[2].split('_')[1];
        document.getElementById(`thruster_${thrusterId}`).style = `--value: ${((data - 1000) / 1000)}`;
        document.getElementById(`thruster_${thrusterId}_pwm`).textContent = `PWM: ${(data.toFixed(1))} μs`;
    }
};