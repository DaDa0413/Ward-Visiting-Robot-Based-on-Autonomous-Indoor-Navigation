const PORT = process.env.PORT || 9000;
// request the express frame
const express = require('express');
const app = express();
// create the peer server
const expressPeerServer = require('peer').ExpressPeerServer;
// set the config of peerserver
const server = app.listen(PORT);
const options = {
    debug: false
};

const peerserver = expressPeerServer(server, options);

app.use('/api', peerserver);

app.get('/', (req, res, next) => { res.send('Hello world!'); });

app.get('/a', function(req, res){
    res.sendFile(__dirname + "/index_A.html");
});

app.get('/b', function(req, res){
    res.sendFile(__dirname + "/index_B.html");
});

peerserver.on('connection', (id) => {
    console.log(`A client connected : ${id}`);
})

peerserver.on('disconnect', (id) => {
    console.log(`A client say ~ bye bye : ${id}`);
});
