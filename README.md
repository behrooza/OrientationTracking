# transform

Quaternion-based orientation tracking for IMU data. The package rotates accelerometer,
gyroscope and magnetometer readings from the sensor (body) frame into a world frame
where gravity points along +Y, using a complementary filter for tilt correction
(accelerometer) and yaw correction (magnetometer).

## Installation

```bash
pip install git+https://github.com/<your-username>/<repo-name>.git
```

## Usage

```python
import numpy as np
import pandas as pd
from transform.transform import transform

t = transform()

# acc, gyr, mag: arrays of shape (N, 3); timestamp: pandas Series in milliseconds
acc_tilt, gyr_tilt = t.transformation_v1(acc, gyr, timestamp, alpha_t=0.98)

acc_tilt, acc_world, gyr_world, mag_world = t.transformation_v2(
    acc, gyr, mag, timestamp, alpha_t=0.98, alpha_y=0.98
)
```

Outputs are arrays of shape (N, 4) in quaternion form `[w, x, y, z]`; the rotated
vector is in columns 1–3.

Magnetometer calibration helpers: `hard_iron`, `soft_iron`, `magnetometer_calibration`.

## License

MIT
