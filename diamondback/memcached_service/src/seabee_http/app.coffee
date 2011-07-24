memcache = require('memcache')
mc = new memcache.Client(11211)
mc.connect()

express = require('express')
app = express.createServer()
io = require('socket.io').listen(app)

app.use(express.static(__dirname + '/public'))
app.listen(80)

MEMCACHE_KEY   = 'seabee3'
sendingUpdates = false
refreshRate    = 500 # in ms

start_sending_updates = (socket) ->
  if not sendingUpdates
    sendingUpdates = true
    setInterval ->
      mc.get(MEMCACHE_KEY,
        (err, response) ->
          if not err?
            data = JSON.parse response
            socket.emit('sensor_data', data)
        )
    ,refreshRate

io.sockets.on('connection',
  (socket) ->
    start_sending_updates(socket)
)