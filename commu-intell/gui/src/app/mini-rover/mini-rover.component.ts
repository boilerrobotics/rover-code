import { Component, OnInit, OnDestroy } from '@angular/core';
import { Subscription } from 'rxjs';
import { MqttService, IMqttMessage } from 'ngx-mqtt';

@Component({
  selector: 'mini-rover',
  templateUrl: './mini-rover.component.html',
  styleUrls: ['./mini-rover.component.css']
})
export class MiniRoverComponent implements OnInit, OnDestroy {

  private subscription: Subscription;
  public message: string;
  latitude: number;
  longitude: number;
  splitedMessage: any;

  constructor(private _mqttService: MqttService) {
    this.subscription = this._mqttService.observe('minirover/status').subscribe((message: IMqttMessage) => {
      this.message = message.payload.toString();
      this.splitedMessage = this.message.split(':');
      this.latitude = (Number)(this.splitedMessage[0]);
      this.longitude = (Number)(this.splitedMessage[1]);
    });
  }

  ngOnInit(): void {
  }

  public ngOnDestroy() {
    this.subscription.unsubscribe();
  }

}
