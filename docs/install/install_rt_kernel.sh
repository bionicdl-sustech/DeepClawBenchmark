# !/bin/sh
wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.14.12/linux-headers-4.14.12-041412_4.14.12-041412.201801051649_all.deb
wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.14.12/linux-headers-4.14.12-041412-generic_4.14.12-041412.201801051649_amd64.deb
wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.14.12/linux-image-4.14.12-041412-generic_4.14.12-041412.201801051649_amd64.deb
sudo dpkg -i *.deb
reboot
