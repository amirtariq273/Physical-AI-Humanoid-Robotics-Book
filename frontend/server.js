const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const ACCOUNTS_FILE = path.join(__dirname, 'accounts.json');

const server = http.createServer((req, res) => {
  // Set CORS headers to allow requests from the frontend
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

  // Handle preflight OPTIONS request for CORS
  if (req.method === 'OPTIONS') {
    res.writeHead(204);
    res.end();
    return;
  }

  // Handle POST request to /signup
  if (req.method === 'POST' && req.url === '/signup') {
    let body = '';
    req.on('data', (chunk) => {
      body += chunk.toString();
    });

    req.on('end', () => {
      try {
        const { email, password } = JSON.parse(body);

        if (!email || !password) {
          res.writeHead(400, { 'Content-Type': 'application/json' });
          res.end(
            JSON.stringify({ message: 'Email and password are required.' })
          );
          return;
        }

        // --- SECURITY WARNING ---
        // In a real application, use a strong, salted hashing algorithm like bcrypt or Argon2.
        // SHA256 is used here for demonstration purposes only.
        const hashedPassword = crypto
          .createHash('sha256')
          .update(password)
          .digest('hex');

        fs.readFile(ACCOUNTS_FILE, 'utf8', (err, data) => {
          if (err && err.code !== 'ENOENT') {
            console.error('Error reading accounts file:', err);
            res.writeHead(500, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ message: 'Internal server error.' }));
            return;
          }

          const accounts = data ? JSON.parse(data) : [];

          // Check if user already exists
          if (accounts.some((acc) => acc.email === email)) {
            res.writeHead(409, { 'Content-Type': 'application/json' });
            res.end(
              JSON.stringify({
                message: 'User with this email already exists.',
              })
            );
            return;
          }

          accounts.push({ email, password: hashedPassword });

          fs.writeFile(
            ACCOUNTS_FILE,
            JSON.stringify(accounts, null, 2),
            (err) => {
              if (err) {
                console.error('Error writing to accounts file:', err);
                res.writeHead(500, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ message: 'Internal server error.' }));
                return;
              }

              res.writeHead(201, { 'Content-Type': 'application/json' });
              res.end(
                JSON.stringify({ message: 'Account created successfully.' })
              );
            }
          );
        });
      } catch {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ message: 'Invalid request body.' }));
      }
    });
  } else {
    res.writeHead(404, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ message: 'Not Found' }));
  }
});

const PORT = 3001;
server.listen(PORT, () => {
  console.log(`Demonstration server running on http://localhost:${PORT}`);
  console.log(
    'This server is for demonstration purposes only and is not secure.'
  );
  // Ensure the accounts file exists
  if (!fs.existsSync(ACCOUNTS_FILE)) {
    fs.writeFileSync(ACCOUNTS_FILE, '[]');
  }
});
