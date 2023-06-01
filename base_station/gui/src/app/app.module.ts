import { IMqttServiceOptions } from 'ngx-mqtt';
export const MQTT_SERVICE_OPTIONS: IMqttServiceOptions = {
  hostname: 'broker.mqttdashboard.com',
  port: 8000,
  protocol: 'ws',
  path: '/mqtt',
};
