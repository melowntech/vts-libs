// mapping between reference frame and map overlay tile URL
// can be external mapping service (lile mapy.cz for ppspace) or boundlayer tile url
var MapMapping = {
    "melown2015": "//cdn.melown.com/mario/proxy/melown2015/tms/melown/bing-world/{z}-{x}-{y}"
    , "ppspace": "//m1.mapserver.mapy.cz/ophoto/{z}_{ppx}_{ppy}"
};

var childTagSize = 20;

var prepareTile = function(layer, coords, generator) {
    var maxCount = Math.pow(2, coords.z);

    // create a <canvas> element for drawing
    var tile = L.DomUtil.create('canvas', 'leaflet-tile');
    // setup tile width and height according to the options
    var size = layer.getTileSize();
    tile.width = size.x;
    tile.height = size.y;

    if ((coords.x < 0) || (coords.x >= maxCount)
        || (coords.y < 0) || (coords.y >= maxCount))
    {
        // out of bounds
        return tile;
    }

    return generator(tile, size, maxCount);
}

var TileGridLayer = L.GridLayer.extend({
    createTile: function(coords) {
        return prepareTile(this, coords, function(tile, size, maxCount) {
            var ctx = tile.getContext('2d');

            ctx.setLineDash([5, 5]);

            // draw grid
            ctx.beginPath();
            if (maxCount == (coords.y + 1)) {
                // last tile
                ctx.moveTo(size.x, size.y);
                ctx.lineTo(0, size.y);
            } else {
                ctx.moveTo(0, size.y);
            }

            ctx.lineTo(0, 0);
            ctx.lineTo(size.x, 0);

            if (maxCount == (coords.x + 1)) {
                // last tile
                ctx.lineTo(size.x, size.y);
            }
            ctx.stroke();
            return tile;
        });
    }
});


var MetaLayer = L.GridLayer.extend({
    initialize: function (url, options) {
        L.GridLayer.prototype.initialize.call(this, options);
        this.url = url;
    },

    createTile: function(coords, done) {
        return prepareTile(this, coords, function(tile, size, maxCount) {
            var url = L.Util.template(this.url, coords);

            var xhr = new XMLHttpRequest();
            xhr.open("GET", url, true);
            xhr.onreadystatechange = function () {
                if (((xhr.readyState) == 4)
                    && ((xhr.status == 200) || (xhr.status == 0)))
                {
                    this.generateTile(tile, size, maxCount, coords, done
                                      , JSON.parse(xhr.responseText));
                } else {
                    done("cannot fetch tile", tile);
                }
            }.bind(this);
            xhr.send(null);
            return tile;
        }.bind(this));
    },

    generateTile: function(tile, size, maxCount, coords, done, node) {
        try {
            var ctx = tile.getContext('2d');

            var center = L.point(size.x / 2, size.y / 2);
            var tileId = coords.z + "-" + coords.x + "-" + coords.y;

            ctx.font = "15px Arial";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(tileId, center.x, center.y);

            var flags = [];

            node.flags.forEach(function(flag) {
                switch (flag) {
                case "ul":
                    this.child(ctx, 0, 0, +childTagSize, +childTagSize);
                    break;
                case "ur":
                    this.child(ctx, size.x, 0, -childTagSize, +childTagSize);
                    break;
                case "ll":
                    this.child(ctx, 0, size.y, +childTagSize, -childTagSize);
                    break;
                case "lr":
                    this.child(ctx, size.x, size.y, -childTagSize, -childTagSize);
                    break;

                default: flags.push(flag); break;
                }
            }.bind(this));

            ctx.fillText(flags.join(","), center.x, center.y + 20);

            // tile is ready
            done(0, tile);
        } catch (error) {
            done(error, tile);
        }
    },

    child: function(ctx, x, y, xoff, yoff) {
        ctx.save();
        ctx.globalAlpha = 0.4;
        ctx.fillStyle = "#00ff00";

        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + xoff, y);
        ctx.lineTo(x, y + yoff);
        ctx.lineTo(x, y);
        ctx.fill();
        ctx.restore();
    }
});

function hex(num, size) {
    var value = num.toString(16);
    while (value.length < size) {
        value = "0" + value;
    }
    return value;
}

function ppx(zoom, x) {
    return hex(x << (28 - zoom), 7)
}

function ppy(zoom, y) {
    return hex((1 << 28) - ((y + 1) << (28 - zoom)), 7)
}

var MapLayer = L.TileLayer.extend({
    getTileUrl: function (coords) {
        // updated function from sources
        var z = this._getZoomForUrl();
        var data = {
            r: L.Browser.retina ? '@2x' : '',
            s: this._getSubdomain(coords),
            x: coords.x,
            y: coords.y,
            z: z,
            ppx: ppx(z, coords.x),
            ppy: ppy(z, coords.y)
        };
        if (this._map && !this._map.options.crs.infinite) {
            var invertedY = this._globalTileRange.max.y - coords.y;
            if (this.options.tms) {
                data['y'] = invertedY;
            }
            data['-y'] = invertedY;
        }

		return L.Util.template(this._url, L.extend(data, this.options));
    }
});


function loadConfig(callback) {
    var xhr = new XMLHttpRequest();
    xhr.overrideMimeType("application/json");
    xhr.open("GET", "debug.json", true);
    xhr.onreadystatechange = function () {
        if (((xhr.readyState) == 4)
            && ((xhr.status == 200) || (xhr.status == 0)))
        {
            callback(JSON.parse(xhr.responseText));
        }
    };
    xhr.send(null);
}


function processConfig(config) {
    var crs = L.CRS.Simple;
    var map = new L.Map("debugger", { crs: crs });

    var startZoom = config.lodRange[0];

    var maskUrl = config.maskUrl.replace(/{lod}/g, "{z}");
    var metaUrl = config.metaUrl.replace(/{lod}/g, "{z}");

    var gridLayer = new TileGridLayer();
    map.addLayer(gridLayer);

    var metaLayer = new MetaLayer(metaUrl);
    map.addLayer(metaLayer);

    var maskLayer = new L.TileLayer(maskUrl, {
        minZoom: 0
        , maxZoom: config.lodRange[1]
        , continuousWorld: true
        , noWrap: true
        , bounds: L.latLngBounds(crs.pointToLatLng(L.point(0, 0))
                                 , crs.pointToLatLng(L.point(256, 256)))
    });

    var mapLayer = null;
    var mapUrl = MapMapping[config.referenceFrame];
    if (mapUrl) {
        mapLayer = new MapLayer(mapUrl, {
            minZoom: 0
            , maxZoom: config.lodRange[1]
            , continuousWorld: true
            , noWrap: true
            , bounds: L.latLngBounds(crs.pointToLatLng(L.point(0, 0))
                                     , crs.pointToLatLng(L.point(256, 256)))
            , opacity: 0.25
        });
    }

    var layers = {};
    var overlays = {};
    overlays["grid"] = gridLayer;
    overlays["meta"] = metaLayer;
    if (mapLayer) { overlays["map"] = mapLayer; }
    layers["mask"] = maskLayer;

    L.control.layers(layers, overlays).addTo(map);
    map.addLayer(maskLayer);

    // set view to center of tile range at min lod
    var center = [ (config.tileRange[0][0] + config.tileRange[1][0]) / 2.0 + 0.5
                   , (config.tileRange[0][1] + config.tileRange[1][1]) / 2.0 + 0.5];

    map.setView(crs.pointToLatLng(L.point(center[0] / Math.pow(2, startZoom - 8)
                                          , center[1] / Math.pow(2, startZoom - 8)))
                , startZoom);
};

function startDebugger() {
    loadConfig(processConfig);
}
