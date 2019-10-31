sudo openvpn --mktun --dev tun1
sudo ifconfig tun1 up
cat "/home/grobots/.cat" | sudo openconnect vpn.clarkson.edu \
	--interface=tun1 \
	-u pacman \
	--reconnect-timeout \
	--passwd-on-stdin
