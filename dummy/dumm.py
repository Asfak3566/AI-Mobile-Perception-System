import requests
from requests.auth import HTTPDigestAuth
import json

# Configuration
url = "http://192.168.6.41/axis-cgi/opticscontrol.cgi"
username = "root"
password = "Inf_MobileMast_1"

# The API requires a JSON body specifying the method "getOptics"
payload = {
    "apiVersion": "1.0",
    "method": "getOptics"
}

headers = {
    "Content-Type": "application/json"
}

try:
    # Send POST request with Digest Authentication
    response = requests.post(
        url,
        json=payload,
        headers=headers,
        auth=HTTPDigestAuth(username, password),
        timeout=10
    )

    if response.status_code == 200:
        data = response.json()
        
        # Extracting the first optics channel (Optics ID 0)
        optics_data = data.get('data', {}).get('optics', [])[0]
        
        zoom_level = optics_data.get('magnification')
        focus_pos = optics_data.get('focusPosition')

        print("--- Camera Reply ---")
        print(f"Zoom (Magnification): {zoom_level}")
        print(f"Focus Position (0-1): {focus_pos}")
        print("\n--- Full JSON ---")
        print(json.dumps(data, indent=2))
        
    else:
        print(f"Error: {response.status_code}")
        print(response.text)

except requests.exceptions.RequestException as e:
    print(f"Connection Error: {e}")
