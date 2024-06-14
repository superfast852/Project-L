sudo apt update && sudo apt -y upgrade
sudo apt -y install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev python3-pip python3-numpy python3-dev cython3
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake -L -DBUILD_PYTHON3=ON -DBUILD_REDIST_PACKAGE=OFF ..
make -j4
sudo make install
sudo ldconfig /usr/local/lib64/

dqt='"'
# ATTR{product}=="Xbox NUI Motor"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02b0${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"
# ATTR{product}=="Xbox NUI Audio"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02ad${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"
# ATTR{product}=="Xbox NUI Camera"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02ae${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"
# ATTR{product}=="Xbox NUI Motor"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02c2${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"
# ATTR{product}=="Xbox NUI Motor"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02be${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"
# ATTR{product}=="Xbox NUI Motor"
sudo su root -c "echo '''SUBSYSTEM==${dqt}usb${dqt}, ATTR{idVendor}==${dqt}045e${dqt}, ATTR{idProduct}==${dqt}02bf${dqt}, MODE=${dqt}0666${dqt}''' >> /etc/udev/rules.d/51-kinect.rules"

clear

mv ./wrappers/python/freenect3.c ../wrappers/python/freenect.c
cd ../wrappers/python
python setup.py build_ext --inplace
sudo cp $(find "./" -name "freenect.cpython-38*" -maxdepth 1) $(python -m site --user-site)
cd ../../../
