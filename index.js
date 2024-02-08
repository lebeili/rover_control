const express = require('express');
const path = require('path');
const bodyParser = require('body-parser');
const { EventEmitter } = require('events');

const app = express();
const port = 3000;

// Create an EventEmitter to handle SSE connections
const eventEmitter = new EventEmitter();

// Middleware to parse JSON data
app.use(bodyParser.json());

// Serve static files (HTML, JS, CSS, etc.)
app.use(express.static(path.join(__dirname, '')));

// Serve the HTML page for the root path
app.get('/rover', (req, res) => {
  res.sendFile(path.join(__dirname, '', 'index.html'));
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
