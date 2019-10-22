sudo openvpn --mktun --dev tun1
sudo ifconfig tun1 up
echo "pacman\nP@cm@n2019!" | \
    sudo openconnect vpn.clarkson.edu --interface=tun1
