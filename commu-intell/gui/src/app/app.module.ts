import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { MiniRoverComponent } from './mini-rover/mini-rover.component';

import { IMqttServiceOptions, MqttModule } from "ngx-mqtt";
import { environment as env } from '../environments/environment';

const MQTT_SERVICE_OPTIONS: IMqttServiceOptions = {
  hostname: env.mqtt.server,
  port: env.mqtt.port,
  path: env.mqtt.path,
};

@NgModule({
  declarations: [
    AppComponent,
    MiniRoverComponent
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
