(function() {
  var MEMCACHE_KEY, app, express, io, mc, memcache, refreshRate, sendingUpdates, start_sending_updates;
  memcache = require('memcache');
  mc = new memcache.Client(11211);
  mc.connect();
  express = require('express');
  app = express.createServer();
  io = require('socket.io').listen(app);
  app.use(express.static(__dirname + '/public'));
  app.listen(80);
  MEMCACHE_KEY = 'seabee3';
  sendingUpdates = false;
  refreshRate = 500;
  start_sending_updates = function(socket) {
    if (!sendingUpdates) {
      sendingUpdates = true;
      return setInterval(function() {
        return mc.get(MEMCACHE_KEY, function(err, response) {
          var data;
          if (!(err != null)) {
            data = JSON.parse(response);
            return socket.emit('sensor_data', data);
          }
        });
      }, refreshRate);
    }
  };
  io.sockets.on('connection', function(socket) {
    return start_sending_updates(socket);
  });
}).call(this);
