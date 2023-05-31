git clone https://github.com/simondlevy/BreezySLAM.git
cd BreezySLAM/python || return 1
python3 setup.py install --user
cd ../..
sudo rm -rf BreezySLAM
pip install -r requirements.txt