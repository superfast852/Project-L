git clone https://github.com/simondlevy/BreezySLAM.git
cd BreezySLAM/python
python3 setup.py install --user
cd ../..
sudo rm -r BreezySLAM
sudo apt install python3-numpy python3-matplotlib python3-scipy python3-pip
pip install -r requirements.txt