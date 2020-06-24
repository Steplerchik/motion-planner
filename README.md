#### Launch all unit tests from a "test" folder
```
python -m unittest discover -s test
```
#### Install motion_planner package
```
./venv/bin/python setup.py install
```
#### Create requirements.txt
```
pip freeze --local > requirements.txt
```
#### Launch a simple test for rrt SE2() planner
```
python test_planner_2.py
```
#### GENERATION of python profile
```
python -m cProfile -o test.profile test_planner_2.py
```
#### INTERPRETING RESULTS of python profile via snakeviz
```
snakeviz test.profile
```