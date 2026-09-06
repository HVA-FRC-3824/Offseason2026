## 3824 2026 Offseason Code

This is the code that the programming team has been working on for the past 3 months over the summer. During this time I've experimented with various vision solutions including both LL and PV for OD and AprilTag localization.

## Project review by directory


### /src/java/frc

This is the path to all of the code that we put

### Misc. Directories

#### /tooling

The tooling directory contains two projects which complement the main robot code project.

##### /tooling/guitar_controller

[My Chief Delphi post](https://www.chiefdelphi.com/t/can-i-use-an-acoustic-guitar-as-a-controller/518013)

This Rust project takes in audio inputs from an acoustic guitar and effectively maps string plucks into virtual controller outputs.

##### /tooling/PayToPlay

[My Chief Delphi post](https://www.chiefdelphi.com/t/does-nfc-count-as-wireless-for-driverstation/522970)

This is both a KMP client app and a server python script that effectively turns NFC detection on a phone into virtual joystick outputs on the DS laptop.

#### /assets

This is our custom assets folder for AdvantageScope. It contains the relevant .glb model files and the .json config. This allows us to have a realistic visualization of our robot in AScope.

#### /pv-calibrations

Where we keep our photonvision calibrations for each of our cameras. Generally, at a competition you'll want to recalibrate each of your cameras for the specific lighting of the field, but this is good enough for testing or when your changing processors.

#### "/External Licenses"

Some code may require that we have their licensure notices presentable in our code.
