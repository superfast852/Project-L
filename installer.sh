sudo apt update && sudo apt -y upgrade
pip install --upgrade pip
git clone https://github.com/simondlevy/breezyslam.git
cd breezyslam/python
python3 setup.py install
cd ../../
pip install -r host-requirements.txt
sudo rm -rf breezyslam
