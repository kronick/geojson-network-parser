const rbush = require('rbush');
const knn = require('rbush-knn');
const turf = require('@turf/turf');
const DijkstraGraph = require('node-dijkstra');

/**
 * A utility module that turns an array of GeoJSON `LineString` features representing a road or path network and turns it into a routable graph.
 * @module GeoJSONNetworkParser
 */

/**
 * Creates a new GeoJSONNetworkParser instance.
 * Stores the GeoJSON features and intiializes data structures but does not parse into a network until {GeoJSONNetworkParser#parse} is called.
 * @constructor
 * @param {Object[]} features - An array of GeoJSON `LineString` features that will be parsed into a network.
 * @param {String} [costProperty] - A property name that will be referenced to calculate the cost of traveling along edges associated with each feature.
 * @param {Number} [normalizeCurves=2] - Simplify and normalize all geometries to have points spaced this number of meters apart.
 */
function GeoJSONNetworkParser(inFeatures, costProperty, normalizeCurves) {
    this.originalFeatures = inFeatures;
    this.segmentTree = rbush();
    this.nodeTree;
    if(normalizeCurves === undefined) normalizeCurves = 2;

    this.lineSegments = this._explodeSegments(normalizeCurves);

    // Put segments into an rtree
    this.segmentTree.load(
        this.lineSegments.map(s => boundingBoxFromSegment(s))
    );

    this.nodes = [];
    this.edges = [];
    this.intersections = [];
    this.edgeGroups = [];
    this.graph = new DijkstraGraph();
    this.costProperty = costProperty;
    this.parsed = undefined;
}

GeoJSONNetworkParser.prototype = {
    /**
     * Find the network nodes closest to a given longitude/latitude. Uses `rbush-knn` for fast search.
     * @param {Number[]} lnglat - An array representing the longitude/latitude of the lookup point.
     * @param {Number} [n=1] - An optional parameter representing the number of results to return.
     * @returns {Object[]} An object representing the nearest node to the input point, or an array of nodes if n > 1.
     */
    getNearestNode: function(lnglat, n) {
        if (n === undefined) n = 1;
        var neighbors = knn(this.nodeTree, lnglat[0], lnglat[1], n);
        neighbors = neighbors.map(n => {
            return {
                node: n.node,
                distance: turf.distance(turf.point(n.node.coordinates), turf.point(lnglat), "meters")
            }
        });
        return n > 1 ? neighbors : neighbors[0];
    },

    /**
     * Find a route between two nodes on the network with the lowest cost using Dijkstra's algorithm.
     * @param {Object} a - The starting node.
     * @param {Object} b - The ending node.
     * @returns {Object} An object with a `nodes` property containing an array of nodes that connect the two points and a `coordinates` property containing an array of longitude/latitude coordinate pairs of each node.
     */
    findShortestPath: function(a, b) {
        var ids = this.graph.path(a.id.toString(), b.id.toString());
        
        if(ids === null) {
            return {nodes: [], coordinates: []}
        }

        var nodes = ids.map(id => { return this.nodes[parseInt(id)] });
        var coordinates = nodes.map(n => { return n.coordinates });
        
        return {nodes: nodes, coordinates: coordinates};
    },

    /**
     * Parse the loaded features into a routable network graph using the supplied parameters to infer topology/connectivity.
     * @param {Object} [options={}] - An object containing parameters used in parsing the network.
     * @param {Number} [options.tolerance=0.00000001] - Tolerance (in degrees latitude) used to determine if two segments connect to the same node. The GeoJSON input features are likely to contain gaps and imprecise connections that will be collapsed when this parameter is set correctly.
     * @param {Object} [options.ignoreCrossings=false] - If true, line segments that cross but do not share a crossing points will not be connected.
     * @returns {Object} An object containing `nodes`, `edges`, and `segments` properties populated with the parsed entities.
     */
    parse: function(options) {
        if(options === undefined) {
            options = {};
        }

        if(options.tolerance === undefined) options.tolerance = 0.00000001;
        if(options.ignoreCrossings === undefined) options.ignoreCrossings = false;

        // Reset
        this.nodeTree = rbush();
        this.edges.length = 0;
        this.edgeGroups.length = 0;
        this.graph = new DijkstraGraph();

        // Pull out all unique nodes from the graph
        this.nodes = this._findNodes(options.tolerance, options.ignoreCrossings);
        // Filter nodes to get intersections and endpoints (order !== 2 i.e. not points along LineStrings)
        this.intersections = this.nodes.filter(n => { return n.edges.length !== 2 });

        
        // Build a list of edges and add them to the graph
        this.nodes.forEach(startingNode => {
            var connections = {};
            startingNode.edges.forEach(startingEdge => {
                var edge = {
                    start: startingNode,
                    end: startingEdge.nodeA === startingNode ? startingEdge.nodeB : startingEdge.nodeA,
                    direction: startingEdge.nodeA === startingNode ? "B" : "A",
                    distance: 0,
                    cost: this.costProperty ? (startingEdge.parent.properties[this.costProperty] || 1) : 1
                }
                
                edge.distance = turf.distance(turf.point(startingEdge.a), turf.point(startingEdge.b), "meters");
                if(edge.distance > 0) {
                    this.edges.push(edge);

                    // Save in format that we can add as a connection in the DijkstraGraph
                    connections[edge.end.id.toString()] = edge.distance * edge.cost;
                }
            });
            // Add this node with its connections to the graph
            this.graph.addNode(startingNode.id.toString(), connections);
        });

        // Walk between all intersections to bin edges into groups for easy interaction
        this.intersections.forEach(startingNode => {
            // Walk from each intersection until we find another intersection
            // Add up length as we walk
            startingNode.edges.forEach((startingEdge) => {
                var nextNode = startingEdge.nodeA === startingNode ? startingEdge.nodeB : startingEdge.nodeA;
                var lastNode = startingNode;
                var nextEdge = startingEdge;
                var edgeGroup = {
                    start: startingNode,
                    segments: [nextEdge],
                    direction: startingNode === startingEdge.nodeB ? "B" : "A",
                    distance: 0,
                    lineStringCoordinates: [startingNode.coordinates, nextNode.coordinates]
                }
                
                // Add a link from the segment back to the parent edge
                if (!startingEdge.parentEdges) startingEdge.parentEdges = [edgeGroup];
                else startingEdge.parentEdges.push(edgeGroup);

                // Add this segment's distance to the total
                edgeGroup.distance += turf.distance(turf.point(startingEdge.a), turf.point(startingEdge.b), "meters");

                while(nextNode.edges.length === 2) {
                    // Not an intersection-- continue on
                    lastNode = nextNode;

                    // Get the next edge and node
                    nextEdge = nextNode.edges[0] === nextEdge ? nextNode.edges[1] : nextNode.edges[0];
                    nextNode = nextEdge.nodeA === nextNode ? nextEdge.nodeB : nextEdge.nodeA;
                    edgeGroup.segments.push(nextEdge);

                    edgeGroup.lineStringCoordinates.push(nextNode.coordinates);

                    // Add a link from the segment back to the parent edge
                    if (!nextEdge.parentEdges) nextEdge.parentEdges = [edgeGroup];
                    else nextEdge.parentEdges.push(edgeGroup);

                    // Add this segment's distance to the total
                    edgeGroup.distance += turf.distance(turf.point(nextEdge.a), turf.point(nextEdge.b), "meters");

                    // Break cyclical rings
                    if(nextNode === startingNode) break;
                }

                edgeGroup.end = nextNode
                edgeGroup.id = this.edgeGroups.length;
                if(edgeGroup.distance > 0)       // Ignore 0 length edges
                    this.edgeGroups.push(edgeGroup);
            });
        });

        this.parsed = {nodes: this.nodes, edges: this.edges, edgeGroups: this.edgeGroups, segments: this.lineSegments};
        return this.parsed;
    },
    _explodeSegments: function(normalizeCurves) {
        var segments = [];
        function getSegments(coordinates, parent) {
            var s = [];
            var distanceSinceLastSegment = 0;
            var lastSegmentEnd = coordinates[0];
            for (var i = 0; i < coordinates.length - 1; i++) {
                distanceSinceLastSegment +=  turf.distance(turf.point(coordinates[i]), turf.point(coordinates[i+1]), "meters");
                if(distanceSinceLastSegment >= normalizeCurves || i === coordinates.length - 2) {
                    var start = lastSegmentEnd;
                    var end = coordinates[i+1];
                    var normalizedSegments = [{
                        a: start,
                        b: end,
                        parent: parent
                    }];

                    var normalizedLength = distanceSinceLastSegment / normalizeCurves;
                    if(normalizedLength > 2) {
                        var split = Math.floor(normalizedLength);
                        normalizedSegments = [];
                        for(var j = 0; j < split; j++) {
                            normalizedSegments.push({
                                a: [start[0] + (end[0] - start[0]) *     j / split, start[1] + (end[1] - start[1]) *     j / split],
                                b: [start[0] + (end[0] - start[0]) * (j+1) / split, start[1] + (end[1] - start[1]) * (j+1) / split],
                                parent: parent
                            });
                        }
                    }
                    
                    s = s.concat(normalizedSegments);
                    distanceSinceLastSegment = 0;
                    lastSegmentEnd = end;
                }
            }

            return s;
        }
        this.originalFeatures.forEach((f) => {
            if(f.geometry.type === "LineString") {
                segments = segments.concat(getSegments(f.geometry.coordinates, f));
                // segments = segments.concat(getSegments(normalized.geometry.coordinates, f));
            }
            else if(f.geometry.type === "Polygon") {
                f.geometry.coordinates.forEach((ring) => {
                    segments = segments.concat(getSegments(ring, f));
                })
            }
        });

        segments.forEach((s, i) => {
            s.id = i;
        });

        return segments;
    },
    _findNodes: function(tolerance, ignoreCrossings) {
        var nodes = [];
        
        var assignEdgesToNode = (point, s1, s2) => {
            var neighbors = this.nodeTree.search({
                minX: point[0] - tolerance,
                maxX: point[0] + tolerance,
                minY: point[1] - tolerance,
                maxY: point[1] + tolerance
            });

            var node;
            if(neighbors.length > 0) { 
                // Sort neighbors by distance
                var sorted = neighbors.sort((a, b) => {
                    var dXA = point[0] - a.minX;
                    var dYA = point[1] - a.minY;
                    var dXB = point[0] - b.minX;
                    var dYB = point[1] - b.minY;
                    var dA = dXA * dXA + dYA + dYA;
                    var dB = dXB * dXB + dYB + dYB;
                    return dA - dB;
                });

                node = sorted[0].node; // Closest match
                if(s1 !== undefined && node.edges.indexOf(s1) === -1)
                    node.edges.push(s1);
                if(s2 !== undefined && node.edges.indexOf(s2) === -1)
                    node.edges.push(s2);

                return node;
            }
            else {
                // No neighbors --> create a new node
                node = {
                    coordinates: point,
                    edges: [s1]
                }

                if(s2 !== undefined) node.edges.push(s2);

                this.nodeTree.insert({
                    minX: point[0],
                    maxX: point[0],
                    minY: point[1],
                    maxY: point[1],
                    node: node
                });

                return node;
            }
        }

        var splitSegment = (segment, atCoords) => {

            const newA = {
                a: segment.a,
                b: atCoords,
                parent: segment.parent
            }

            const newB = {
                a: atCoords,
                b: segment.b,
                parent: segment.parent
            }

            // Add new segments to the list of segments and the tree index
            this.lineSegments.push(newA);
            this.lineSegments.push(newB);
            this.segmentTree.insert(boundingBoxFromSegment(newA));
            this.segmentTree.insert(boundingBoxFromSegment(newB));

            // TODO: Remove old segment from the index
            // this.segmentTree.remove(boundingBoxFromSegment(segment), function (a, b) {
            //     return a.segment === b.segment;
            // });

            // TODO: Remove node references for old segment

            return [newA, newB];
        };

        var upsertNode = (n) => {
            if(nodes.indexOf(n) === -1) {
                n.id = nodes.length;
                nodes.push(n);
            }
        }

        var extendSegment = (segmentFeature) => {
            const s = segmentFeature.geometry.coordinates;
            const P = [s[1][0] - s[0][0], s[1][1] - s[0][1]];
            const magP = Math.sqrt(P[0] * P[0] + P[1] * P[1]);
            const Pnorm = [P[0] / magP, P[1] / magP];
            return {
                type: "Feature",
                geometry: {
                    type: "LineString",
                    coordinates: [
                        [s[0][0] - Pnorm[0] * tolerance, s[0][1] - Pnorm[1] * tolerance],
                        [s[1][0] + Pnorm[0] * tolerance, s[1][1] + Pnorm[1] * tolerance]
                    ]
                }
            }
        }

        if(!ignoreCrossings) {
            // Loop through all segments once, looking for intersections and splitting segments but not actually adding nodes yet.
            this.lineSegments.forEach((s1) => {
                var aMatched = false;
                var bMatched = false;
                var neighbors = this.segmentTree.search(boundingBoxFromSegment(s1, tolerance));
                //neighbors = neighbors.map((n) => { return n.segment });


                neighbors.forEach((neighbor) => {
                    var s2 = neighbor.segment;
                    if(s1 === s2) return;

                    // 1. Check endpoints
                    if (nearEnough(s1.a, s2.a, tolerance) || nearEnough(s1.a, s2.b, tolerance)) {
                        aMatched = true;
                    }
                    else if (nearEnough(s1.b, s2.b, tolerance) || nearEnough(s1.b, s2.a, tolerance)) {
                        bMatched = true;
                    }
                    else {
                        var seg1 = {type: "Feature", geometry: {type: "LineString", coordinates: [s1.a, s1.b]}};
                        var seg2 = {type: "Feature", geometry: {type: "LineString", coordinates: [s2.a, s2.b]}};
                        seg1 = extendSegment(seg1);
                        seg2 = extendSegment(seg2);
                        const intersections = turf.lineIntersect(seg1, seg2);
                        if(intersections !== undefined && intersections.features.length > 0 && intersections.features[0].geometry.type === "Point") {
                            const intersection = intersections.features[0];
                            if(nearEnough(s1.a, intersection.geometry.coordinates, tolerance)) {
                                // T-intersection: Split s2, assign new two new edges and s1 to node at s1.a
                                const newSegments = splitSegment(s2, s1.a);
                                aMatched = true;
                            }
                            else if(nearEnough(s1.b, intersection.geometry.coordinates, tolerance)) {
                                // T-intersection: Split s2, assign new two new edges and s1 to node at s1.b
                                const newSegments = splitSegment(s2, s1.b);
                                bMatched = true;
                            }
                            else if(nearEnough(s2.a, intersection.geometry.coordinates, tolerance)) {
                                // T-intersection: Split s1, assign new two new edges and s2 to node at s2.a
                                const newSegments = splitSegment(s1, s2.a);
                                // var n = assignEdgesToNode(s2.a, newSegments[0], newSegments[1]);
                                // upsertNode(n);
                            }
                            else if(nearEnough(s2.b, intersection.geometry.coordinates, tolerance)) {
                                // T-intersection: Split s1, assign new two new edges and s2 to node at s2.b
                                const newSegments = splitSegment(s1, s2.b);
                                // var n = assignEdgesToNode(s2.b, newSegments[0], newSegments[1]);
                                // upsertNode(n);
                            }
                            else {
                                // X-intersection: Split s1 and s2, assign four new edges to new node at intersection point
                                const newSegments1 = splitSegment(s1, intersection.geometry.coordinates);
                                const newSegments2 = splitSegment(s2, intersection.geometry.coordinates);
                                
                            }
                        }
                    }
                });
            });
        }


        // A. Add nodes between all segments (even those lying in the middle of a polyline)
        this.lineSegments.forEach((s1) => {
            var aMatched = false;
            var bMatched = false;
            var neighbors = this.segmentTree.search(boundingBoxFromSegment(s1, tolerance));
            //neighbors = neighbors.map((n) => { return n.segment });

            neighbors.forEach((neighbor) => {
                var s2 = neighbor.segment;
                if(s1 === s2) return;

                // 1. Check endpoints
                if (nearEnough(s1.a, s2.a, tolerance) || nearEnough(s1.a, s2.b, tolerance)) {
                    aMatched = true;
                    var n = assignEdgesToNode(s1.a, s1, s2);
                    s1.nodeA = n;
                    upsertNode(n);
                }
                else if (nearEnough(s1.b, s2.b, tolerance) || nearEnough(s1.b, s2.a, tolerance)) {
                    bMatched = true;
                    var n = assignEdgesToNode(s1.b, s1, s2);
                    s1.nodeB = n;
                    upsertNode(n);
                }
            });

            // 3. If a or b endpoint isn't matched, add in as a node anyway
            if(!aMatched) {
                var n = assignEdgesToNode(s1.a, s1);
                s1.nodeA = n;
                if(nodes.indexOf(n) === -1) {
                    n.id = nodes.length;
                    nodes.push(n);
                }
            }
            if(!bMatched) {
                var n = assignEdgesToNode(s1.b, s1);
                s1.nodeB = n;
                if(nodes.indexOf(n) === -1) {
                    n.id = nodes.length;
                    nodes.push(n);
                }
            }
        });

        return nodes;
    }
}


function nearEnough(a, b, tolerance) {
    var dX = a[0] - b[0];
    var dY = a[1] - b[1];
    var dist = Math.sqrt(dX * dX + dY * dY);
    return dist <= tolerance;
}

function boundingBoxFromSegment(s, buffer) {
    if(buffer === undefined) buffer = 0;
    return {
        minX: Math.min(s.a[0], s.b[0]) - buffer,
        maxX: Math.max(s.a[0], s.b[0]) + buffer,
        minY: Math.min(s.a[1], s.b[1]) - buffer,
        maxY: Math.max(s.a[1], s.b[1]) + buffer,
        segment: s
    }
}

module.exports = exports = GeoJSONNetworkParser;