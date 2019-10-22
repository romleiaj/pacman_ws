sudo openvpn --mktun --dev tun1
sudo ifconfig tun1 up
sudo openconnect vpn.clarkson.edu --interface=tun1
