import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';

import { MqttModule, IMqttServiceOptions } from 'ngx-mqtt';
export const MQTT_SERVICE_OPTIONS: IMqttServiceOptions = {
  hostname: 'broker.mqttdashboard.com',
  port: 8000,
  protocol: 'ws',
  path: '/mqtt'
}

@NgModule({
  declarations: [
    AppComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    MqttModule.forRoot(MQTT_SERVICE_OPTIONS)
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
