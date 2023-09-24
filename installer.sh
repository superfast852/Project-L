pip install --upgrade pip || return 1
git clone https://github.com/simondlevy/BreezySLAM.git
cd BreezySLAM/python || return 1
python3 setup.py install
cd ../../Resources/py_install/
python setup.py install
cd ../../
pip install -r requirements.txt