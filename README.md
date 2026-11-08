# transform

Quaternion-based orientation tracking for IMU data. The package rotates accelerometer,
gyroscope and magnetometer readings from the sensor (body) frame into a world frame
where gravity points along +Y, using a complementary filter for tilt correction
(accelerometer) and yaw correction (magnetometer).

## Installation

```bash
pip install git+https://github.com/behrooza/OrientationTracking.git
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

## Background

The orientation filter follows the drift-correction approach (gyroscope integration
with accelerometer tilt correction and magnetometer yaw correction) originally described in:

- S. M. LaValle, A. Yershova, M. Katsev, M. Antonov. "Head tracking for the Oculus Rift."
  *2014 IEEE International Conference on Robotics and Automation (ICRA)*, pp. 187–194, 2014.
  [doi:10.1109/ICRA.2014.6906608](https://doi.org/10.1109/ICRA.2014.6906608)

The method as implemented in this package is explained in:

- B. Azadi, M. Haslgrübler, A. Ferscha. "Motion Analysis in Alpine Skiing: Sensor Placement
  and Orientation-Invariant Sensing." *Sensors*, 25(8), 2582, 2025.
  [doi:10.3390/s25082582](https://doi.org/10.3390/s25082582)

It is used as a preprocessing step in:

- B. Azadi, M. Haslgrübler, B. Anzengruber-Tanase, S. Grünberger, A. Ferscha. "Alpine Skiing
  Activity Recognition Using Smartphone's IMUs." *Sensors*, 22(15), 5922, 2022.
  [doi:10.3390/s22155922](https://doi.org/10.3390/s22155922)
- B. Azadi, M. Haslgrübler, B. Anzengruber-Tanase, G. Sopidis, A. Ferscha. "Robust Feature
  Representation Using Multi-Task Learning for Human Activity Recognition." *Sensors*, 24(2),
  681, 2024. [doi:10.3390/s24020681](https://doi.org/10.3390/s24020681)

## Citation

If you use this package, please cite:

```bibtex
@article{azadi2025motion,
  author  = {Azadi, Behrooz and Haslgr{\"u}bler, Michael and Ferscha, Alois},
  title   = {Motion Analysis in Alpine Skiing: Sensor Placement and Orientation-Invariant Sensing},
  journal = {Sensors},
  volume  = {25},
  number  = {8},
  pages   = {2582},
  year    = {2025},
  doi     = {10.3390/s25082582}
}
```

## License

MIT
