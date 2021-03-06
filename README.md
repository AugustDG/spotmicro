# spotmicro
Arduino code repo for my build of the Spot Micro project, more specifically Michael's Kubina version! I am using 2 ultrasonic sensors (not implemented as of yet), the PCA 9685 PWM driver, 12 MG995 servos and an Arduino UNO!

You can find the original repo with all the instructions and model files [here](https://github.com/michaelkubina/SpotMicroESP32) :)

Otherwise, have fun reading over my code! Can't say it's perfect, but it's for a school project and if you have any suggestions don't hesitate to raise an issue or send a pull request :D

I am also developing a companion app (in WPF) that goes with this code to control the bot [here](https://github.com/AugustDG/spotmicro-companion-windows) :)

You should always calibrate your servos and change the PWM value ranges accordingly :)

If you're ever finding the new folders/files bizarre, it's because I'm trying out the Arduino IDE 2.0!

The important file is the *.ino* one, so that shouldn't change 👌🏽

## Dependencies

* [NewPing](https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home)
* [Adafruit PWM Servo Driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

## Servo Arrangement

I've arranged the servos in the below fashion, the [🧑🏽] emoji is the front and the [🍑] is the rear of the bot :)

🧑🏽

| Right | Left |
|:---:|:---:|
| 2 | 1 |
| 3 | 4|

🍑


## State of Things

| Features | Working? | Somewhat Optimized? |
| :--- | :---: | :---: |
| Degrees -> PWM values | ✔ | ✔ |
| Uniform movement from all servos | ✔ | ✔ |
| Relative 2D Coordinates -> Degrees (IK) | ✔ | ❌ |
| Step Routine (take a step without falling) | ✔ | ❌ |
| Walking Routine (walks various steps without falling) | ✔ | ❌ |

MIT Licensed :)