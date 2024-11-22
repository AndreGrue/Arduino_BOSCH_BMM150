`Arduino_BOSCH_BMM150`
=====================

This is an Arduino library for the [BOSCH BMM150](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150) magnetometer 
like it is used in the [Arduino Nano 33 BLE Sense](https://www.st.com/en/development-tools/nano-33-ble-sense.html).
The library supports several ways of reading the sensor data,

This library uses the [BOSCH BMM150 Sensor API](https://github.com/boschsensortec/BMM150_SensorAPI).

## Data sheets
- [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf)

## Usage

### Object creation

```cpp
#include "bosch_bmm150.h"
andrgrue::sensor::bosch_bmm150 Mag(Wire1);

float mag_x, mag_y, mag_z;
andrgrue::sensor::bosch_bmm150::Data mag_data;
```

### Setup

```c++
void setup() {

  // wire must be initialized first
  Wire1.begin();
  Wire1.setClock(400000);

  if(!Mag.initialize(BMM150_DATA_RATE_25HZ)) {
    Serial.println("MAG init failed!");
    while (1);
  }

  Serial.print("sample rate: mag=");
  Serial.println(Mag.magneticfieldSampleRate());
}
```

### Loop

```c++
void loop() {

  if(Mag.magneticfieldAvailable()){
    Mag.magneticfield(mag_data);
  }

}
```

## Credits

This project was inspired and includes several elements from the following projects.
Special thanks to the authors.

 - [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)

## License

Copyright © 2024, André Grüttner. All rights reserved.

This project is licensed under the GNU Lesser General Public License v2.1 (LGPL-2.1).
You may use, modify, and distribute this software under the terms of the LGPL-2.1 license.
See the [LICENSE](./LICENSE.txt) file for details, or visit [GNU’s official LGPL-2.1 page](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html) for the full license text.
