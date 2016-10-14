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
                }
            }.bind(this));

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

function loadFreeLayer(callback) {
    var xhr = new XMLHttpRequest();
    xhr.overrideMimeType("application/json");
    xhr.open("GET", "freelayer.json", true);
    xhr.onreadystatechange = function () {
        if (((xhr.readyState) == 4)
            && ((xhr.status == 200) || (xhr.status == 0)))
        {
            callback(JSON.parse(xhr.responseText));
        }
    };
    xhr.send(null);
}


function processFreeLayer(fl) {
    var crs = L.CRS.Simple;
    var map = new L.Map("debugger", { crs: crs });

    var startZoom = fl.lodRange[0];

    var maskUrl = fl.meshUrl.replace(/{lod}/g, "{z}")
        .replace(/bin/g, "mask.dbg");

    var metaUrl = fl.metaUrl.replace(/{lod}/g, "{z}")
        .replace(/meta/g, "meta.dbg");

    var gridLayer = new TileGridLayer();
    map.addLayer(gridLayer);

    var metaLayer = new MetaLayer(metaUrl);
    map.addLayer(metaLayer);

    var maskLayer = new L.TileLayer(maskUrl, {
        minZoom: 0
        , maxZoom: fl.lodRange[1]
        , noWrap: true
    });

    var layers = {};
    var overlays = {};
    overlays["grid"] = gridLayer;
    overlays["meta"] = metaLayer;
    layers["mask"] = maskLayer;

    L.control.layers(layers, overlays).addTo(map);
    map.addLayer(maskLayer);

    // set view to center of tile range at min lod
    var center = [ (fl.tileRange[0][0] + fl.tileRange[1][0]) / 2.0 + 0.5
                   , (fl.tileRange[0][1] + fl.tileRange[1][1]) / 2.0 + 0.5];

    map.setView(crs.pointToLatLng(L.point(center[0] / Math.pow(2, startZoom - 8)
                                          , center[1] / Math.pow(2, startZoom - 8)))
                , startZoom);
};

function startDebugger() {
    loadFreeLayer(processFreeLayer);
}
