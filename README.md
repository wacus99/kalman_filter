# kalman-filter

The project was made for statistics course at university. Its goal was to simulate one of simplest
mobile robot's model. In this case robot was driving only in straight line. System input was its acceleration
and observed state was velocity.

## Usage

To start simulation run `kalman.py`

Each sensor has its parameters inside class.
Class `DummySensor` can be used to represent any sensor with given noise covariance and sample time.

## Conclusions

`IMU`, `GPS` and `Encoder` sensor implementation doesn't work as it is intended for usage with Kalman filter.
They were designed to show thinks such as accelerometer drift. Sensor that works as it is intended for Kalman filter is `DummySensor` which have true white noise with known standard deqiation.
