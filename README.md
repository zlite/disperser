# M5Stack Bug Disperser

Firmware for an M5Stack Basic Core and the **Module13.2 Stepmotor Driver
(M039)**. It controls a CoreXY plotter and a third stepper that lowers a tray
into a water bath.

> This project targets the pulse-driven M039 module at I2C address `0x27`.
> It does not target the M035 GRBL module. If the module is marked `GRBL`, do
> not run this firmware.

## Controls

- **Button A — Home:** homes Z to L0, physical X to L1, and physical Y to L2,
  assigns each switch position as axis zero, then parks XY at the provisional
  center target `(100 mm, 100 mm)`. Its directions are derived as the inverse of
  the corresponding homing directions so that both axes leave their switches.
- **Button B — Start/Stop:** starts the cycle after homing; during motion it
  requests a controlled stop, returns XY to the swish center, and raises Z.
- **Button C — Pause/Resume:** immediately pauses or resumes step generation.

Motor holding current is disabled after homing/parking and after a cycle has
fully stopped. It remains enabled during Pause so the tray and carriage retain
their known positions.

Each timed cycle records the current XY position, lowers the tray, repeats the
selected motion until the configured duration expires, returns to center, and
raises Z automatically. Wi-Fi status, machine state, and position are shown on
the built-in LCD.

## Web controls

Once connected to `Archive_01`, open [http://disperser.local/](http://disperser.local/)
from a device on the same network. The dashboard mirrors Home, Start/Stop, and
Pause/Resume and shows live state, position, elapsed time, and L0 status.

The dashboard settings apply on the next Start:

- **Motion:** circular, back-and-forth, side-to-side, or combination.
- **XY speed:** 0.5–10 mm/s.
- **Duration:** 5–300 seconds of active swishing time; paused time is excluded.
- **Radius/travel:** 0.5–10 mm around the XY position captured at Start.

Circular motion alternates clockwise and counterclockwise revolutions.
Combination runs both circle directions followed by both linear patterns.

## Required wiring

Connect the two CoreXY motors to the module's X and Y motor outputs and the tray
motor to Z. Connect active-low (switch-to-ground) limits as follows:

| Module input | Function |
| --- | --- |
| L0 / P0 | Z upper limit |
| L1 / P1 | Physical X home limit |
| L2 / P2 | Physical Y home limit |

The firmware uses the M5Stack-documented Basic Core pins:

| Motor | STEP | DIR |
| --- | ---: | ---: |
| CoreXY A / module X | GPIO 16 | GPIO 17 |
| CoreXY B / module Y | GPIO 12 | GPIO 13 |
| Tray Z | GPIO 15 | GPIO 0 |

## Before the first powered move

1. Turn motor power off before plugging or unplugging any motor.
2. In [`include/config.h`](include/config.h), verify motor direction inversion,
   steps/mm, maximum Z homing travel, swish radius, and Z lowering distance.
3. Move the mechanism away from every hard stop and initially test with the
   tray removed from the bath.
4. Be ready to remove motor power. Press Home and verify that Z travels **up**.
   If not, power off and change `INVERT_MOTOR_Z`.
5. Confirm L0, L1, and L2 stop their respective homing moves before testing Start.
6. Verify the CoreXY directions during a dry cycle and reverse the relevant
   motor direction settings if the circle motion is wrong.

The default `80` XY steps/mm assumes 200 full steps/revolution, 1/8
microstepping, a 20-tooth pulley, and 2 mm GT2 belt pitch. Z uses the proven
Entosieve baseline of 100 steps/cm (10 steps/mm), 50 steps/second, and a 10 us
STEP pulse. The 50 mm XY park distances are provisional; measure the resulting
physical travel to calibrate both steps/mm and the true machine center.

## Build and upload

Install [PlatformIO](https://platformio.org/), connect the M5Stack Basic Core,
copy the secrets template and enter the local Wi-Fi credentials:

```sh
cp include/secrets.example.h include/secrets.h
```

Then run:

```sh
pio run
pio run --target upload
pio device monitor
```

Motion does not depend on Wi-Fi remaining connected. The configured network is
used only for connectivity/status in this version.
