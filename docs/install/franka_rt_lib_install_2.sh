# !/bin/sh
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)

sudo sh -c "echo @realtime soft rtprio 99 >> /etc/security/limits.conf"
sudo sh -c "echo @realtime soft priority 99 >> /etc/security/limits.conf"
sudo sh -c "echo @realtime soft memlock 102400 >> /etc/security/limits.conf"
sudo sh -c "echo @realtime hard rtprio 99 >> /etc/security/limits.conf"
sudo sh -c "echo @realtime hard priority 99 >> /etc/security/limits.conf"
sudo sh -c "echo @realtime hard memlock 102400 >> /etc/security/limits.conf"

