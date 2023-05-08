cd Packages/BreezySLAM
python3 setup.py install --user
cd ../rrtplanner
python3 setup.py install --user
cd ../..
sudo apt install python3-numpy python3-matplotlib python3-scipy python3-pip libopenblas-dev gfortran
pip install -r requirements.txt
pip install --no-deps scikit-image