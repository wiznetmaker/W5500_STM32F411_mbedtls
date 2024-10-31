# How to Test TCP Client over SSL Example



## Step 1: Prepare software

The following serial terminal program and SSL server are required for TCP Client over SSL example test, download and install from below links.

- Tera Term
- OpenSSL


## Step 2: Setup TCP Client over SSL Example

1. Setup network configuration such as IP in 'main.c'.

Setup IP and other network settings to suit your network environment.

```cpp
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};
```

2. Setup TCP Client over SSL configuration in 'main.c'.

In the TCP client over SSL configuration, the target IP is the IP of your desktop or laptop where SSL server will be created.

```cpp
/* Port */
#define PORT_SSL 443

static uint8_t g_ssl_target_ip[4] = {192, 168, 11, 3};
```

In order to change SSL settings, modify 'ssl_config.h' located in the same directory with TCP Client over SSL example.


## Step 3: Upload and Run

1. Connect to the serial COM port of Raspberry Pi Pico, W5100S-EVB-Pico, W5500-EVB-Pico, W55RP20-EVB-Pico, W5100S-EVB-Pico2 or W5500-EVB-Pico2 with Tera Term.

![][link-connect_to_serial_com_port]

2. Run OpenSSL to be used as the SSL server.

3. Create SSL server using openSSL by executing the following command. If the SSL server is created normally, the SSL server's IP is the current IP of your desktop or laptop, and the port is 443 by default.

```cpp
/* Setup the SSL server */
// create the private key
genrsa -des3 -out [key name].key 2048
// create the CSR(required for certificate signing request)
req -new -key [key name].key -out [csr name].csr
// create the certificate
x509 -req -days [days] -in [csr name].csr -signkey [key name].key -out [crt name].crt

// e.g.
genrsa -des3 -out server.key 2048
req -new -key server.key -out server.csr
x509 -req -days 365 -in server.csr -signkey server.key -out server.crt

/* Run the SSL server */
s_server -accept [port] -cert [crt name].crt -key [key name].key

// e.g.
s_server -accept 443 -cert server.crt -key server.key
```

4. Reset your board.


## Appendix

- mbedTLS library was ported to use SSL, please refer to following link to find version of ported mbed TLS.
