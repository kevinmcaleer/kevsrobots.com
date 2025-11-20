# Proxy Headers Configuration

## Problem

When running behind proxies (Cloudflare → Nginx → FastAPI), the application sees the proxy IP (192.168.1.4) instead of the real client IP.

## Solution

The application now extracts the real client IP from proxy headers in this priority order:

1. **X-Forwarded-For** - Contains chain of IPs (original client is first)
2. **X-Real-IP** - Set by Nginx with original client IP
3. **request.client.host** - Fallback (will be proxy IP)

---

## Nginx Configuration

### Load Balancer Configuration

Your Nginx load balancer should forward these headers:

**File**: `/etc/nginx/sites-available/search-api` (or your nginx config)

```nginx
upstream search_backend {
    least_conn;
    server 192.168.2.11:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.12:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.13:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.14:8000 max_fails=3 fail_timeout=30s;
}

server {
    listen 80;
    server_name search.kevsrobots.local;

    location / {
        proxy_pass http://search_backend;

        # IMPORTANT: Forward client IP headers
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # Additional headers
        proxy_set_header X-Forwarded-Host $host;
        proxy_set_header X-Forwarded-Port $server_port;

        # Timeouts
        proxy_connect_timeout 60s;
        proxy_send_timeout 60s;
        proxy_read_timeout 60s;
    }
}
```

### Key Headers Explained

| Header | Value | Purpose |
|--------|-------|---------|
| `X-Real-IP` | `$remote_addr` | Single IP of the client |
| `X-Forwarded-For` | `$proxy_add_x_forwarded_for` | Chain of IPs (client, proxy1, proxy2) |
| `X-Forwarded-Proto` | `$scheme` | Original protocol (http/https) |
| `X-Forwarded-Host` | `$host` | Original hostname |

---

## Cloudflare Configuration

Cloudflare automatically adds these headers when proxying:

- **CF-Connecting-IP** - Original client IP
- **X-Forwarded-For** - Chain including Cloudflare IPs
- **True-Client-IP** - Original client (Enterprise plan only)

Our app uses **X-Forwarded-For** which is available on all Cloudflare plans.

---

## Verify Headers

### 1. Check Current Headers

Add this temporary endpoint to your app to see all headers:

```python
@app.get("/debug/headers")
async def debug_headers(request: Request):
    """Debug endpoint to see all headers (remove in production!)"""
    return {
        "client_host": request.client.host if request.client else "none",
        "headers": dict(request.headers),
        "computed_ip": get_client_ip(request)
    }
```

Then test:
```bash
curl http://your-domain.com/debug/headers
```

Look for these headers:
- `x-forwarded-for`
- `x-real-ip`
- `cf-connecting-ip` (if behind Cloudflare)

### 2. Test From Different IPs

```bash
# From your local machine
curl "https://www.kevsrobots.com/search/?query=test"

# Check the logs
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs -c \
  "SELECT timestamp, client_ip, query FROM search_logs ORDER BY timestamp DESC LIMIT 5;"
```

You should see different IPs for different clients, not 192.168.1.4 for everyone.

---

## Troubleshooting

### All IPs still show as 192.168.1.4

**Cause**: Nginx is not forwarding headers properly.

**Solution**:

1. Check Nginx config includes proxy headers:
```bash
grep -r "X-Forwarded-For" /etc/nginx/sites-enabled/
```

2. Reload Nginx:
```bash
sudo nginx -t
sudo systemctl reload nginx
```

3. Verify headers are being forwarded:
```bash
# Add debug endpoint and check
curl http://localhost:8000/debug/headers
```

### IPs show as Cloudflare IPs

**Cause**: X-Forwarded-For contains Cloudflare IPs in the chain.

**Example**: `X-Forwarded-For: 203.0.113.1, 104.16.0.1`
- `203.0.113.1` = Real client
- `104.16.0.1` = Cloudflare proxy

**Solution**: The app already handles this by taking the **first IP** in the chain.

### Getting "unknown" as IP

**Cause**: None of the headers are present and request.client is None.

**Solution**: Check that your web server is properly connected and headers are forwarded.

---

## Testing Configuration

### Test Script

Create `test_ip_logging.sh`:

```bash
#!/bin/bash

echo "Testing IP logging..."

# Make a search request
RESPONSE=$(curl -s "https://www.kevsrobots.com/search/?query=test_$(date +%s)")

# Wait a moment for logging
sleep 1

# Check the database
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs -c \
  "SELECT
    timestamp,
    client_ip,
    query
   FROM search_logs
   ORDER BY timestamp DESC
   LIMIT 5;"
```

Run it:
```bash
chmod +x test_ip_logging.sh
./test_ip_logging.sh
```

### Expected Output

```
           timestamp            |   client_ip    |      query
--------------------------------+----------------+-----------------
 2025-11-20 15:30:45.123456    | 203.0.113.1    | test_1700489445
 2025-11-20 15:29:12.654321    | 198.51.100.42  | robot
 2025-11-20 15:28:03.111222    | 192.0.2.33     | micropython
```

Notice different IPs for different clients.

---

## Security Considerations

### IP Spoofing

When trusting proxy headers, be aware that clients can spoof these headers if they connect directly to your app.

**Protection**:

1. **Firewall**: Ensure only Nginx can connect to FastAPI ports
```bash
# Only allow from Nginx server
sudo ufw allow from 192.168.1.4 to any port 8000
sudo ufw deny 8000
```

2. **Trusted Proxies**: In production, validate the X-Forwarded-For chain

3. **Network Isolation**: Run FastAPI on private network, only Nginx on public

### Rate Limiting

With correct IPs logged, you can implement rate limiting per client:

```python
from collections import defaultdict
from time import time

# Simple rate limiter (use Redis in production)
request_counts = defaultdict(list)

def rate_limit(ip: str, max_requests: int = 100, window: int = 60) -> bool:
    """Check if IP exceeds rate limit"""
    now = time()
    # Clean old requests
    request_counts[ip] = [req_time for req_time in request_counts[ip]
                          if now - req_time < window]

    if len(request_counts[ip]) >= max_requests:
        return False

    request_counts[ip].append(now)
    return True

@app.get("/search/")
async def search_documents(request: Request, ...):
    client_ip = get_client_ip(request)

    if not rate_limit(client_ip):
        raise HTTPException(status_code=429, detail="Too many requests")

    # ... rest of endpoint
```

---

## Nginx Configuration Update Script

Create `update_nginx_headers.sh`:

```bash
#!/bin/bash

NGINX_CONFIG="/etc/nginx/sites-available/search-api"

echo "Checking Nginx configuration..."

if ! grep -q "X-Forwarded-For" "$NGINX_CONFIG"; then
    echo "WARNING: X-Forwarded-For header not found in config"
    echo "Add these lines to your location block:"
    echo ""
    echo "    proxy_set_header X-Real-IP \$remote_addr;"
    echo "    proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;"
    echo "    proxy_set_header X-Forwarded-Proto \$scheme;"
    echo ""
else
    echo "✓ X-Forwarded-For header is configured"
fi

if ! grep -q "X-Real-IP" "$NGINX_CONFIG"; then
    echo "WARNING: X-Real-IP header not found in config"
else
    echo "✓ X-Real-IP header is configured"
fi

echo ""
echo "Testing Nginx config syntax..."
sudo nginx -t

if [ $? -eq 0 ]; then
    echo ""
    read -p "Reload Nginx to apply changes? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo systemctl reload nginx
        echo "✓ Nginx reloaded"
    fi
fi
```

---

## Monitoring

### Track IP Diversity

Check if you're getting varied IPs:

```sql
-- Unique IPs in last 24 hours
SELECT
    COUNT(DISTINCT client_ip) as unique_ips,
    COUNT(*) as total_searches,
    ROUND(COUNT(DISTINCT client_ip)::numeric / COUNT(*)::numeric, 2) as diversity_ratio
FROM search_logs
WHERE timestamp > NOW() - INTERVAL '24 hours';
```

**Healthy diversity**: ratio > 0.1 (10% unique IPs)
**Problem**: ratio < 0.01 (all requests from same IP)

### Most Active IPs

```sql
-- Top 10 IPs by search count
SELECT
    client_ip,
    COUNT(*) as search_count,
    COUNT(DISTINCT query) as unique_queries,
    MIN(timestamp) as first_seen,
    MAX(timestamp) as last_seen
FROM search_logs
WHERE timestamp > NOW() - INTERVAL '7 days'
GROUP BY client_ip
ORDER BY search_count DESC
LIMIT 10;
```

---

## After Deployment

1. **Deploy updated app** with `get_client_ip()` function
2. **Verify Nginx headers** are configured
3. **Test from multiple locations** (home, mobile, VPN)
4. **Check database** for varied IPs
5. **Remove debug endpoint** if you added one
6. **Monitor** IP diversity over time

Your searches should now log real client IPs instead of the proxy IP!
