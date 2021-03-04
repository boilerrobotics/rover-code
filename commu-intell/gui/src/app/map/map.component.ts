import { Component, OnInit } from '@angular/core';

import Map from 'ol/Map';
import TileLayer from 'ol/layer/Tile';
import View from 'ol/View';
import OSM from 'ol/source/OSM';
import * as olProj from 'ol/proj';

@Component({
  selector: 'gps-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css']
})
export class MapComponent implements OnInit {

  map: Map;
  target_longitude: number = 7.0785;
  target_latitude: number = 7.0785;

  ngOnInit(): void {
    this.map = new Map({
      target: 'test_map',
      layers: [
        new TileLayer({
          source: new OSM()
        })
      ],
      view: new View({
        center: olProj.fromLonLat([
          this.target_longitude, 
          this.target_latitude
        ]),
        zoom: 17
      })
    });
  }

}
