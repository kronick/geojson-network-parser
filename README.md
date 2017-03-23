# A utility that turns a GeoJSON FeatureCollection of a road network and turns it into a routable graph

## Installation

`npm install geojson-network-parser`

## Usage

```
const NetworkParser = require('./NetworkParser');


var geojson = {
    type: 'FeatureCollection',
    features: [
        {
            type: 'Feature',
            properties: {
                layer: 'roadway',
                cost: 4
            },
            geometry: {
                type: 'LineString',
                geometry: [[-37.231312, 32.4444], [-37.23111, 32.5534] /* and so on ... */ ]
            }
        },
        // ... and so on with more roads
    ]
};

// Filter the features only process the ones that represent roads
const roads = geojson.features.filter(f => (f.properties.layer === "roadway"));

// Turn the GeoJSON FeatureCollection of roads into a network that we can find shortest-path routes on
const network = new NetworkParser(roads, "cost", 2);
const parsed = network.parse({
  tolerance: 0.0000075,   // Ignore gaps greater than this distance. Units are in degrees latitude, so values < 0.00002 are a good starting point.
  ignoreCrossings: false  // If `true`, intersections will only be added where there are two nearby points in the original FeatureCollection. If `false`, intersections will be inferred where two edge segments cross each other.
}); 


// Snap latitude/longitude point to nearest node on the network
const A = network.getNearestNode([-37.231312, 32.4444]);
const B = network.getNearestNode([-37.23111, 32.5534]);

// Get a list of nodes that connect the shortest path between two points
const route = network.findShortestPath(A, B);

```

## Building from source

Clone this repo then run:

```
npm install
npm run build
```

This will output a bundled js file to `dist/geojson-network-parser.js`.
