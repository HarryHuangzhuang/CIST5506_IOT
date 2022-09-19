#include <pgmspace.h>

#define THINGNAME "OccupancyMonitor_01"

const char WIFI_SSID[] = "Lemar's Galaxy S22";
const char WIFI_PASSWORD[] = "vofl1662";
const char AWS_IOT_ENDPOINT[] = "a3buxc2awep6h3-ats.iot.ap-southeast-2.amazonaws.com";

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAKy7JB1ByNvTgXzVCIDwK2M8N2B5MA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMjA5MTkwNDA3
MjRaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDUz4+KU6rPpKV4rjrh
Gqfzav1r1j18JOd8OWkvZFccXfGH8A3raWLNxZs5Jr/nNjAuMDm76fgH1zmN4ayi
dPZLMHB8zMTVNmDL4rIlNDD7u8X+mg5vY0xMBglP92obVU9s4VzxOc/N78+yvzfq
WNotuizMDQy5tEl5DZSuxf1Yi3RZq3Zm3YwsCreOftwGNAfQ1ST6ZrMXKZkjM0pk
30CglF/fMEdA2dSzMzU0b78jmZLJUY/ab46+ArIX7WBZz8dl1P9hVtw4Dgzvht5S
2XKujWmfSy7dhsg/vKkcHcG0eT6dw6Ff+H9KohVZvLkOC1ISBXtmBhDD6/pBC2Yn
X//FAgMBAAGjYDBeMB8GA1UdIwQYMBaAFHJj7OR/gU6mCHMNK2Y37rodEU7fMB0G
A1UdDgQWBBSwaPAeGKR5DAx+kEpXEeTBnKqfgzAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAtIagmN5hLZFilKrk08X0Ztf8
Oav3nPikgLgi4x3eDz6yzBY1DIYs+hTt3e/5fY430VOe5uR3XikPZuGdGkm5rs1A
a0p3cTlLigR6YVRhFE4RgAagl+nQ8iz+rmj67pvwKCj1eFB5APLl4zCttA62jyyn
BJMbkTv2nKot+ppPJqCR8vPx3lq1yHbS4sw5NJchYVLShk987565VAyftwfuvNyc
lvgGz3R4w7YSEFy/l777CrQXiubPieNAjHUcBbku0Uhhk3wAkKP5FRTCF/Mc29ty
yG6BLKUcdcML5ahfs59l/UF/brsa1P0ca3cnG9WwaBTbyHRYp5Zzd9AN88gaKg==
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEA1M+PilOqz6SleK464Rqn82r9a9Y9fCTnfDlpL2RXHF3xh/AN
62lizcWbOSa/5zYwLjA5u+n4B9c5jeGsonT2SzBwfMzE1TZgy+KyJTQw+7vF/poO
b2NMTAYJT/dqG1VPbOFc8TnPze/Psr836ljaLboszA0MubRJeQ2UrsX9WIt0Wat2
Zt2MLAq3jn7cBjQH0NUk+mazFymZIzNKZN9AoJRf3zBHQNnUszM1NG+/I5mSyVGP
2m+OvgKyF+1gWc/HZdT/YVbcOA4M74beUtlyro1pn0su3YbIP7ypHB3BtHk+ncOh
X/h/SqIVWby5DgtSEgV7ZgYQw+v6QQtmJ1//xQIDAQABAoIBAH7sODEJC5QCH8ql
J2rASjEMu46IXW1gWUumtgxU7Pt3MCmiS7r+xLXzSEvCx6uvjvbNPnHL7mlGvQXY
9eyCk65ueniEbFS5tOuFDjjQMvLAoCrkWP2oHIYvB3jRGFCrBc7rblJU55Jsl9SE
5WV4js+8/SDKG6/aTgPLzMScQyh+FVVi4dOM+D3CeCH6A2vpj9oURspYTHVE8aeX
sfGQIUVGnG7R55IOu5uvlR1rQbrrBwrVwBhofBVA4x9ryeh99pnIBXmCXZCZgZhN
8Ci3Mh2b40LuyCfb0Llbr5Jv56/bXcsjYxFdaZugIUHAk5MAhkaTcuPo2f9Qay2f
6ng4z3UCgYEA8ahCHuQl8lCLUxZofAyKSzaAQoHcCwgQ2Oaey+tYMERv8OqBkP33
k1MD3TIA6YFWYpoHqtcGqgpesyfQRYNeUH40MVyslz6dUj8A/5CNzEGn6rLbfZ3b
AccUIvmau8rkqNt1rk0otMMc+ejP6kyQ+yEzNhWIlQuYKJLEq+R5ew8CgYEA4XEC
SMB0O/qQUtdrvQY8+Ioj8O9DgQvl3KNl8yilvilA5+27MwQoRzcSkQ455Nw6H71m
oK5yKF3B30aS+VDGhamYBSywqvTtYmxb3yyBnMrXpPvd2rCQlaMdsDOOmGFZwn7K
iFg2qFqXlzN6kbr5PaIAAmVaHC525RmHt7MXZ+sCgYBDGToKw8R5jL2yu1UVpJG9
ootobBBCC+JJp3dOHbGl7Uq7tRF2xd25TWKUR28TZvMqe6RHXl8eVbTHv/VHZ9uH
NBOf5SCS+jLKhG3tiCCnJ0/Njg39yhfmXXWM5It/Oy3UaOM7G3eDAW4JQgiBAPN8
l1JV4RLEd6yvD1mGdPD6JwKBgG97Cr4fLVtblHLixAw9QXlGFm/WYXM3BvwMcSXF
O/DFRatrC982dxSAT1YbytU5ciSv7EgY+6qqcmyO4a8YGpOOQfaNGu/vHlRFcUjz
IUT/qZOUWSkDlvvoJY40cJc5zME5Ib46oQyvf7btZfcalpW7vnVw2RaGg5TXAx38
Gd7xAoGAb6c+NKF/84L4KgFGV3u8n3DI0kRLGe9gZHzY9OvaAzPHF8uEZ2GUHY1n
rOhabjwUDz0zSjhtcrjmvrzFpiSzIrL0baj0TMReN2enIVTb+ns0s+KA61i8WP1R
oNCGHY23ELMRRwPtaqFregrL81IGk1ms6W6HrecqeWz13LVcKvk=
-----END RSA PRIVATE KEY-----
)KEY";