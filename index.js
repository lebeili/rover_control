const express = require('express');
const basicAuth = require('express-basic-auth');
const path = require('path');
const bodyParser = require('body-parser');
const { EventEmitter } = require('events');

const app = express();
const port = 80;
const users = { 'blabla': '228228' };
// Create an EventEmitter to handle SSE connections
const eventEmitter = new EventEmitter();

// Middleware to parse JSON data
app.use(bodyParser.json());

const basicAuthMiddleware = basicAuth({
    users: users,
    challenge: true, // Show pop-up window for authentication
    unauthorizedResponse: 'Unauthorized Access!', // Custom unauthorized response
  });
  
  // Use basicAuthMiddleware for the path you want to protect
  app.use('/rover', basicAuthMiddleware);
  


// Serve static files (HTML, JS, CSS, etc.)
app.use(express.static(path.join(__dirname, '')));

// Serve the HTML page for the root path
app.get('/rover', (req, res) => {
  res.sendFile(path.join(__dirname, '', 'rover.html'));
});

// SSE endpoint to send real-time updates
app.get('/events', (req, res) => {
  res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');

  // Listen for updates
  const updateListener = (data) => {
    res.write(`data: ${JSON.stringify(data)}\n\n`);
  };

  eventEmitter.on('update', updateListener);

  // Remove the listener when the client disconnects
  req.on('close', () => {
    eventEmitter.off('update', updateListener);
  });
});

// API endpoint to receive data from the client
app.post('/api/updateData', (req, res) => {
    console.log("connection tried");
  const newData = req.body;
  
  // Emit an event to update connected clients
  eventEmitter.emit('update', newData);

  res.json({ success: true });
});

app.listen(port, () => {
  console.log(`Server listening at http://localhost:${port}`);
});


//const WebSocket = require('ws');
//const server = new WebSocket.Server({ port: 3000 }); // Replace with your desired port

/*server.on('connection', (socket) => {
  console.log('Client connected');

  socket.on('message', (message) => {
	     let jsonString = message.toString('utf8');
    let jsonData = JSON.parse(jsonString);
    console.log('Received message from client:', jsonData);
	ws.send(jsonData);
    // Handle the received JSON object as needed
  });

  socket.on('close', () => {
    console.log('Client disconnected');
  });
});*/

console.log('WebSocket server is running on port 3000');
