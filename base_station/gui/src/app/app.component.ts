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
  private forDebug!: Subscription; // For print out all incoming messages 
  public message!: string;

  // Need to use onConnect to check connection status

  constructor(private _mqttService: MqttService) { 
    this.forDebug = this._mqttService.observe('brc/#')
    .subscribe((message: IMqttMessage) => {
      this.message = message.payload.toString();    
      console.log(`Recieved ${this.message} from topic` +
        `${message.topic.toString()}`
      );
    });
  }

  ngOnDestroy(): void {
    this.forDebug.unsubscribe();
  }

}
