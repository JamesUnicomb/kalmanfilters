# kalmanfilters

## Install

```
cd kalmanfilters
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
pip install .
```

## Run Examples

### Run full 6dof EKF with accelerometer, gyroscope and magnetometer readings

```
pip install -r examples/requirements.txt
python examples/cvekf/example1.py
python examples/cvekf/example2.py
```

### Run full 6dof EKF realtime with accelerometer, gyroscope and magnetometer readings

```
pip install -r examples/requirements.txt
python examples/run_compass.py
python examples/run_kf.py
```

[![Extended Kalman Filter](https://img.youtube.com/vi/3IVuh-_xESg/0.jpg)](https://www.youtube.com/watch?v=3IVuh-_xESg)

## Fully Embedded Application

[![Extended Kalman Filter](https://img.youtube.com/vi/1sdPbJsYebA/0.jpg)](https://www.youtube.com/watch?v=1sdPbJsYebA)

## Run Tests

Currently TODO.

```
pip install -r test/requirements.txt
pytest test -rP
```