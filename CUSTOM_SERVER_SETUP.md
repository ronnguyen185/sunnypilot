# Custom Server Upload Setup

This guide explains how to configure openpilot to upload routes to your own server instead of comma.ai's servers.

## Quick Setup (Option 1: Environment Variable)

The simplest way is to set the `API_HOST` environment variable:

```bash
export API_HOST=https://your-server.com
```

Your server must implement the same API endpoint structure as comma.ai:
- `GET /v1.4/{dongle_id}/upload_url/?path={file_path}`
- Returns JSON: `{"url": "https://...", "headers": {...}}`
- Accepts JWT authentication: `Authorization: JWT {token}`

## Advanced Setup (Option 2: Custom API Class)

A custom API class has been created at `common/api/custom_server.py` that allows more flexibility.

### 1. Configure Your Server URL

Edit `openpilot/common/api/custom_server.py` and set your server URL:

```python
CUSTOM_API_HOST = os.getenv('CUSTOM_API_HOST', 'https://your-server.com')
```

Or set it via environment variable:
```bash
export CUSTOM_API_HOST=https://your-server.com
```

### 2. Customize Upload Endpoint (Optional)

If your server uses a different endpoint structure, set the `UPLOAD_ENDPOINT` environment variable:

```bash
export UPLOAD_ENDPOINT=api/upload/url
```

The uploader will call: `GET {CUSTOM_API_HOST}/{UPLOAD_ENDPOINT}?path={file_path}`

### 3. Server Requirements

Your server needs to:

1. **Accept JWT Authentication**
   - Token is sent in header: `Authorization: JWT {token}`
   - Token is signed with device's private key (RS256 algorithm)
   - Token payload contains: `identity` (dongle_id), `iat`, `exp`, `nbf`

2. **Return Upload URL**
   - Endpoint: `GET /v1.4/{dongle_id}/upload_url/?path={file_path}` (or your custom endpoint)
   - Response format:
     ```json
     {
       "url": "https://storage.example.com/path/to/file",
       "headers": {
         "Content-Type": "application/octet-stream",
         "Authorization": "Bearer ..."
       }
     }
     ```

3. **Accept File Upload**
   - The returned URL should accept HTTP PUT requests
   - Files may be compressed (`.zst` format)
   - Headers from the response should be included in the PUT request

### 4. Authentication Options

If your server doesn't use JWT, you can modify `custom_server.py`:

```python
def api_get(self, endpoint, method='GET', timeout=10, access_token=None, json=None, **kwargs):
    headers = {}
    # Use API key instead of JWT
    api_key = os.getenv('CUSTOM_API_KEY')
    if api_key:
        headers['Authorization'] = f'Bearer {api_key}'
    # ... rest of implementation
```

### 5. Testing

To test without actually uploading:
```bash
export FAKEUPLOAD=1
```

## Example Server Implementation

Here's a minimal example of what your server endpoint should do:

```python
from flask import Flask, request, jsonify
import jwt
from datetime import datetime, timedelta

app = Flask(__name__)

@app.route('/v1.4/<dongle_id>/upload_url/', methods=['GET'])
def get_upload_url(dongle_id):
    # Verify JWT token
    auth_header = request.headers.get('Authorization')
    if not auth_header or not auth_header.startswith('JWT '):
        return jsonify({'error': 'Unauthorized'}), 401
    
    token = auth_header[4:]  # Remove 'JWT ' prefix
    # Verify token (you'll need the public key)
    # ... verification code ...
    
    # Get file path from query parameter
    file_path = request.args.get('path')
    
    # Generate signed URL for your storage (S3, Azure, etc.)
    upload_url = generate_signed_url(dongle_id, file_path)
    headers = {
        'Content-Type': 'application/octet-stream',
        # Add any required headers for your storage service
    }
    
    return jsonify({
        'url': upload_url,
        'headers': headers
    })

def generate_signed_url(dongle_id, file_path):
    # Implement your storage service's signed URL generation
    # Examples: AWS S3, Azure Blob Storage, Google Cloud Storage
    return "https://your-storage.com/path/to/file"
```

## File Structure

Routes are uploaded with the following structure:
- `{route_name}/{segment_num}/qlog` - Main log file
- `{route_name}/{segment_num}/qlog.zst` - Compressed log
- `{route_name}/{segment_num}/qcamera.ts` - Camera video
- `boot/{bootlog_name}.zst` - Boot logs
- `crash/{crashlog_name}` - Crash logs

Route names are in format: `{dongle_id}|{timestamp}`


