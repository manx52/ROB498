
### Optional VNC setup
- 192.168.0.101 is the jetson ip. It should be different for each Jetson so run ifconfig to find it.
```bash
ssh rob498@192.168.0.101 -C -L 5900:localhost:5900 # Port forwarding
sudo apt install x11vnc

# Start VNC
x11vnc -forever -xkb -auth guess -display :0 -localhost -nopw

# Or add as an alias:
echo "alias vnc='x11vnc -forever -xkb -auth guess -display :0 -localhost -nopw'" >> .bashrc
source .bashrc
vnc # starts the vnc server, needs to be run every

# Then every on personal computer:
sudo apt-get install xtightvncviewer
vncviewer 0.0.0.0:5900
```

Setup your ssh-keys
```bash
ssh-keygen # Then keep entering, don't set a password
cd ~/.ssh &&  cat id_rsa.pub # Get contents of public key
# on github > Settings > SSH and GPG Keys > New SSH Key, paste the key
```

Setup auto ssh on personal computer, this will make it so you don't need to enter the password every time
```
ssh-copy-id rob498@jetson_ip
```
