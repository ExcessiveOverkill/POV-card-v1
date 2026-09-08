# Persistence of Vision Badge
Turn on then shake side to side to view the images or animations programmed into the badge.

## Specs:
Requires 2x CR2032 batteries (badge is not rechargeable)
Credit card sized: 2.1" x 3.3" (54mm x 84mm)
0.8mm PCB thickness, 4mm total thickness with components
Black PCB soldermask
32 bright white leds
ENIG (thin gold) plated

## Features:
Image or animation display
User programmable images/animations over USB-C
User configurable modes set by tapping the badge
4-bit grayscale image support
Many new display modes
XYZ Accelerometer
True power-off (batteries last when not in use)
Auto power-off


# How to use:
## Turning On:
Press "ON" button on badge.
Badge will display power-on animation when first powered on.

Pressing the "ON" button once badge is already on will have no effect.

## Turning Off:
Do nothing

Badge will automatically power off after it is no longer being interacted with for around 20 seconds. It will also power off regardless of interaction after 5 minutes.

## Changing Mode:
1. Turn badge on
2. Tap top of badge downward 3 times quickly to enter the mode menue, it should display the current mode by turning on an LED. The distance the LED is from the bottom of the badge shows what mode it is in.
3. Tap top of badge downward to move LED down, Tap bottom of badge upward to move LED up
4. Wait 3 seconds for badge to enter new mode. You may shake the badge while in the mode menu to view what mode is selected.
5. Done!

## USB Programming:
1. Turn badge on
2. Plug into computer using a USB-C cable, ensure the cable can carry data and not only power.
3. Badge will display slow pulsing to indicate USB connection.
4. Badge should show up as a USB storage device (flash drive).
5. Copy your BMP file to the badge
6. Done!
   
### BMP File Requirements:
- 1-bit or 4-bit grayscale
- 8, 16, or 32 pixels tall
- 128 or less pixels wide (per frame)
- Filename must end in "1" or "2" to indicate which slot you want to program
- For animations (multiple frames displayed in sequence), the filename must start with "SEQ"
- Animation frames must be separated by a 1 pixel wide column of alternating colors (0-1 for 1-bit, 0-15 for 4-bit). With the top pixel starting as 1 or 15 (white).
- File size must be less than 2KB

## How to get a good image
Image quality greatly depends on the shaking motion. Use mode 8: "POV Line Test" to visualize your shake motion.
### Things to improve display quality
- Hold badge further away
- Smooth, uniform shaking. Slower and more consistent is better than fast and crazy.
- Shake left/right, not tilted or up/down
- Symmetrical movement, *not* like trying to shake ketchup out of a bottle.

## Modes
### 1. POV Display All
Display both the pre-programmed image/animation shipped with the badge as well as all user-programmed images/animations.

### 2. POV Display User
Display only the user-programmed images/animations.

### 3. Level
Emulates a bubble level. Hold the badge edge against a vertical or horizontal surface. Use the "Level Calibration" mode to improve accuracy.

### 4. Ball Simulator
LEDs display a "ball" that reacts to how you move and tilt the badge.

### 5. Save Mode
Save the last selected mode and automatically switch to it on the next power-on.
This function requires verification (see below).

### 6. Level Calibration
Calibrate the angles the badge uses for "Level" mode. After entering this mode, hold the long badge still against a known horizontal surface until the LEDs all turn on. Then hold the short edge still against the surface until the "bubble" resets to center.
This function requires verification (see below).

### 7. Reset
Clears all user-programmed memory. Badge will power off after this mode is selected.
This function requires verification (see below).

### 8. POV Line Test
Displays a diagonal line. Useful for practicing the correct shake technique to get the best display results.
The line should be perfectly straight and vertical when you shake the badge. Slight distortion is acceptable, but if it is very distorted, read the "How to get a good image" section.

## Firmware Update (DANGER)
### 9. USB Device Firmware Update
Allows the badge to be fully reprogrammed with a new binary file using STM32CubeProgrammer over the USB port. `-ob nBOOT0=1` must be added when using the programmer to reset the DFU bit, otherwise the badge will be stuck in DFU mode. Once this mode is entered, the badge will not function until reprogrammed! "ON" button must be held down to program the badge in this mode *or* with an ST-LINK.
User-memory will also be cleared.


## Mode Verification
Certain modes are protected by a verification step to prevent accidental triggering.
These modes will display a bar at the bottom the image that slowly shrinks while you shake the badge.
You must continuously shake the badge until the bar is gone to enable the selected mode.


# Warnings/Notices:
- The badge is a bare PCB, it is very susceptible water and metallic objects and dust. Keep clean and do not put in contact with loose conductive objects
- Badge is not rechargeable, the USB-C port does not supply power to the badge circuitry.
- This "Development Edition" is the first publicly available version. There may be unforeseen issues with hardware or software that affect functionality. However we will do our best to verify all use cases and resolve potential issues before you receive your badge.
- Persistence-of-vision effects are notoriously hard to film. Cameras will not capture the effect well without a very slow shutter speed.
- Attached lanyards or clips that rattle against the badge will affect the accelerometer reading and can make the image distorted.
