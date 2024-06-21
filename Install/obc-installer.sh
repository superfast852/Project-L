# TODO: it'd probably be responsible to activate a venv

sudo apt install python3-pip git python-is-python3
sudo usermod -aG input $(whoami)
sudo usermod -aG dialout $(whoami)

pip install --upgrade pip
sudo pip3 install -U jetson-stats
sudo pip3 install -U Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -aG gpio $(whoami)
sudo cp ./99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

git clone https://github.com/simondlevy/breezyslam.git
cd breezyslam/python
python3 setup.py install

cd ../../
rm -rf breezyslam
pip install -r obc-requirements.txt
python3 create_startup.py
pip install --upgrade scipy
pip install --upgrade -r obc-requirements.txt