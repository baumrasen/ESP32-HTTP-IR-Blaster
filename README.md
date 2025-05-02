# Fork

This is a fork base on https://github.com/mdhiggins/ESP8266-HTTP-IR-Blaster

You will find hardware and software related things there. For example, how to build a JSON with some IR codes.

# Differences
For this fork I used an D1 Mini ESP32. Maintarget was, to get a webbased remotecontrol. 

# Setup

I used VS Code with PlatformIO.

# Example

## Blumfeldt / Smart Beam

<img width="250" src="https://raw.githubusercontent.com/baumrasen/ESP8266-HTTP-IR-Blaster/refs/heads/master/remote_example.png">

Command	Type	Length	Address

1 On/Off
FF4AB5	NEC	32	0x0

2 Adjustment button (left - higher)
FF827D	NEC	32	0x0

2 Adjustment button (right - lower)
FF22DD	NEC	32	0x0

3 High/Low switch button
FF728D	NEC	32	0x0

4 Switch button
FF926D	NEC	32	0x0

5 Set button
FF02FD  NEC     32      0x0

6 Display button
FF12ED	NEC	32	0x0

```
         _______________________
        |                       |
        |           ●           | <- IR-Sensor
        |                       |
        |  _________________    |
        | |                 |   |
        | |   (⭘)    [|||] |   | <- 1: Power     3: High/Low switch button
        | |_________________|   |
        |                       |
        |                       |
        |                       |
        |                       |
        |     _____________     |
        |    /             \    |
        |   /     SWITCH    \   | <- 4: SWITCH (oben)
        |  |     _______     |  |
        |  |    |       |    |  |
        |  | ↑  |  SET  |  ↓ |  | <- 2: Hoch / Runter, 5: SET (Mitte)
        |  |    |_______|    |  |
        |  |                 |  |
        |  |      DISPLAY    |  | <- 6: DISPLAY (unten)
        |   \               /   |
        |    \_____________/    |
        |                       |
        |                       |
        |_______________________|

Legende:
(1) Power-Taste (oben links)
(3) Signal-/Modus-Taste (oben rechts)
(2) Richtungstasten: Links (◄), Rechts (►)
(4) SWITCH-Taste (oben im Kreis)
(5) Rechte Richtungstaste
(6) DISPLAY-Taste (unten im Kreis)
[SET] Zentrale Taste zum Bestätigen
```

## Button config
See the file ./buttons_example.json
Import it from the webinterface.