const express = require('express');
const bodyParser = require('body-parser');
const fs = require('fs');
const path = require('path');

const app = express();
const port = 3000;

// Use body-parser middleware to parse request bodies
app.use(bodyParser.json({ limit: '10mb' }));

// Serve HTML page
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});
// Serve JPEG files based on the requested path
app.get('/frames/:imageName', (req, res) => {
    const imageName = req.params.imageName;
    const imagePath = path.join(__dirname, 'frames', imageName);

    // Send the JPEG file
    res.sendFile(imagePath);
});
// Handle frame upload
app.post('/upload', (req, res) => {
    // Decode base64 image and save it to a file
    const imageData = req.body.image;
    const imageBuffer = Buffer.from(imageData, 'base64');

    // Save the image to a file (you can customize this part)
    const fileName = `latest.jpg`;
    fs.writeFileSync(path.join(__dirname, 'frames', fileName), imageBuffer);

    // Respond with success message
    res.status(200).send('Frame received successfully!');
});

app.listen(port, () => {
    console.log(`Server listening at http://localhost:${port}`);
});

