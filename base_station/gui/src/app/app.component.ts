import { Component, OnDestroy } from '@angular/core';
import { IMqttMessage, MqttService } from 'ngx-mqtt';
import { Subscription } from 'rxjs';
import { RouterOutlet } from '@angular/router';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css'],
  standalone: true,
  imports: [RouterOutlet],
})
export class AppComponent implements OnDestroy {
  private _title = 'Boiler Robotics';
  private _subscription!: Subscription;
  public message!: string;

  constructor(private _mqttService: MqttService) {
    this._subscription = this._mqttService
      .observe('brc')
      .subscribe((message: IMqttMessage) => {
        this.message = message.payload.toString();
        console.log(this.message);
      });
  }

  ngOnDestroy(): void {
    this._subscription.unsubscribe();
  }
}
