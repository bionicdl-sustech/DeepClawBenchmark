# !/bin/sh
# https://frankaemika.github.io/docs/
sudo apt-get install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex

wget -c https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
wget -c https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
wget -c https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
wget -c https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign

xz -d linux-4.14.12.tar.xz
xz -d patch-4.14.12-rt10.patch.xz

gpg2 --verify linux-4.14.12.tar.sign
sudo gpg2  --keyserver hkp://keys.gnupg.net --recv-keys 0x6092693E
sudo gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x2872E4CC
gpg2 --verify linux-4.14.12.tar.sign
sudo gpg2 --verify patch-4.14.12-rt10.patch.sign

tar xf linux-4.14.12.tar
cd linux-4.14.12
patch -p1 < ../patch-4.14.12-rt10.patch

make oldconfig
fakeroot make -j4 deb-pkg
sudo dpkg -i ../linux-headers-4.14.12-rt10_*.deb ../linux-image-4.14.12-rt10_*.deb
reboot

