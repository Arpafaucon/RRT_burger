# Network

## Setup

The Turtlebot3 has a Wi-Fi card, while the local PC computer has 2 Ethernet adapters.

##### Network topology

To give the robot a perfect freedom of movement, the experiment setup is as follows:
```
ACCESS : INRIA internet access 
    | Ethernet
    | DHCP - dynamic IP
CLIENT : local PC 
    | Ethernet
    | assigned IP : 10.0.0.2
ACCESS : router (hidden SSID, login tomate/cornichon, DHCP on 10.0.0.0/24)
    | Wi-Fi
    | assigned IP : 10.0.0.4
CLIENT : Turtlebot
```

##### Internet Access
A direct consequence of this setup is that **only the local PC has an internet access**, in accordance to the rules of the INRIA lab.

##### IP assignment

Both clients expect the IP addresses to be given by the router. The configuration for reserved IP is in the router, in the `DHCP` section.

##### Modified files

The Wi-Fi credentials are in Turtlebot's `/etc/network/interfaces` file.

## Troubleshoot

##### Invalid time

The DHCP won't properly work if devices have incoherent time. To solve this potential issue: 
- **Router** : System Tools > Date & Time > click `get from PC`(that normally has a valid time)
- **Turtlebot** : <code>sudo date --s "2017-12-05T16:06:56-0100</code>

#### ssh - Connection refused 

Re-enable the launch of sshd (which runs the ssh server) at startup by running : 

```systemctl enable ssh.socket```

## Setup an additional computer

It is possible to work with another computer in that setup, but that requires a special configuration to be able to access the Turtlebot & internet.
Here is a description of a working solution (Ubuntu 16.04 LTS)

### Connections : basic setup

It is possible to connect via wifi or ethernet to the experiment router. In this case, the router and the main PC will be reachable (with ping or ssh), but the device won't have any internet access (the router was set up without internet for security reasons).
A more complex setup allows the device to also have an internet access : see below.

#### Connections : complex setup

The computer is connected in WiFi to `eduroam`, and to the router by an ethernet cable.

### Identify interfaces

```sh
ip link show
```
result : 
```
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: enp2s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN mode DEFAULT group default qlen 1000
    link/ether 58:8a:5a:28:08:95 brd ff:ff:ff:ff:ff:ff
3: wlp3s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP mode DORMANT group default qlen 1000
    link/ether e4:70:b8:11:2d:26 brd ff:ff:ff:ff:ff:ff
```
We identify the ethernet interface `enp2s0` and the WiFi inteface `wlp3s0`.

### Identify default gateway

We need to know the ip address of the gateway of the network with Internet (in this example: `wlp3s0`).
```
ip route show
```
result
```
default via 192.168.1.1 dev wlp3s0  proto static  metric 600 
169.254.0.0/16 dev wlp3s0  scope link  metric 1000 
192.168.1.0/24 dev wlp3s0  proto kernel  scope link  src 192.168.1.24  metric 600
...
```
The gateway address is in a line beginning by **default via** and mentioning **dev wlp3s0**. 

Here : `192.168.1.1`

### Checking and editing routes

With the default configuration, the Linux kernel tends to privilege one interface over the other (usually the Ethernet one, because bandwith is faster ). We will tell the kernel to use Wlan for everything **_except_** for the robot subnet (for which Ethernet should be used). In short :
- robot subnet (subnet 10.0.0/24)-> use `enp2s0`
- Internet (everything else) -> use `wlp3s0`

Note: below command require root rights

Note: replace `192.168.1.1` by the gateway address found above.

```sh
# clearing the current routing table
ip route flush
# adding a route to the experiment subnet via Ethernet
ip route add 10.0.0/24 dev enp2s0
# adding a default route to internet via wlan
ip route add default dev wl3ps0 via 192.168.1.1
```

### Troubleshooting
To see the current routes, use
```sh
ip route show
```

To test how a given route is resolved
```sh
# Using the IP of salade
ip route get 10.0.0.2
```