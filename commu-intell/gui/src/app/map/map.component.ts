import { Component, OnInit } from '@angular/core';

import * as olsource from 'ol/source';
import Map from 'ol/Map';
import TileLayer from 'ol/layer/Tile';
import View from 'ol/View';
import Feature from 'ol/Feature';
import { Point } from 'ol/geom';
import { Icon, Style } from 'ol/style';
import { fromLonLat } from 'ol/proj';
import { Vector } from 'ol/layer';




@Component({
  selector: 'gps-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css']
})
export class MapComponent implements OnInit {

  map: Map;
  layer: Vector;
  target_longitude: number = -86.91617;
  target_latitude: number = 40.421679;

  ngOnInit(): void {
    this.map = new Map({
      target: 'test_map',
      layers: [
        new TileLayer({
          source: new olsource.OSM()
        })
      ],
      view: new View({
        center: fromLonLat([
          this.target_longitude, 
          this.target_latitude
        ]),
        zoom: 16
      })
    });

    this.layer = new Vector({
      source: new olsource.Vector({
        features: [
          new Feature({
            geometry: new Point(fromLonLat([
              this.target_longitude, 
              this.target_latitude
            ]))
          })
        ]
      })
    });
    // this.layer.setStyle(
    //   new Style({
    //     image: new Icon({
    //       color: '#ffcd46',
    //       crossOrigin: 'anonymous',
    //       src: 'dot.png',
    //     })
    //   })
    // );
    this.map.addLayer(this.layer);
  }

}
