// Minimal service worker placeholder. Kept intentionally simple for local robot dashboard.
self.addEventListener('install', function (event) { self.skipWaiting(); });
self.addEventListener('activate', function (event) { event.waitUntil(self.clients.claim()); });
