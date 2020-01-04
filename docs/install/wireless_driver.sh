# !/bin/sh
lspci | grep -i net
#sudo apt-get install --reinstall git dkms build-essential linux-headers-$(uname -r)
sudo apt-get install git dkms build-essential linux-headers-$(uname -r)
git clone https://github.com/tomaspinho/rtl8821ce
cd rtl8821ce
chmod +x dkms-install.sh 
chmod +x dkms-remove.sh
sudo ./dkms-install.sh
reboot
