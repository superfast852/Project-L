# TODO: it'd probably be responsible to activate a venv

pip install --upgrade pip
git clone https://github.com/simondlevy/breezyslam.git
cd breezyslam/python
python3 setup.py install
cd ../../
pip install -r obc-requirements.txt