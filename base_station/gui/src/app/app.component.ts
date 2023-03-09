import { Component, OnDestroy, OnInit } from '@angular/core';
import { IMqttMessage, MqttService } from 'ngx-mqtt';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})
export class AppComponent implements OnDestroy{
  title = 'Boiler Robotics';
  private subscription!: Subscription;
  public message!: string;

  constructor(private _mqttService: MqttService) { 
    this.subscription = this._mqttService.observe('brc')
    .subscribe((message: IMqttMessage) => {
      this.message = message.payload.toString();
      console.log(this.message);
    });
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }

}
