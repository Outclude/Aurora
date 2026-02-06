const express = require('express');
const path = require('path');
const app = express();
const port = 2333;

// Serve static files from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Fallback for any other route
app.get(/(.*)/, (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

app.listen(port, () => {
    console.log(`Pacer Laser Belt Controller running at http://localhost:${port}`);
});
