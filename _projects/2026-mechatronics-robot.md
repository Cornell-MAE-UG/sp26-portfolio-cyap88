---
layout: project
title: MAE 3780 Final Project
description: Robot Built and Programmed as Part of Mechatronics Final Competition
technologies: [C, Arduino IDE]
image: /assets/images/boe-bot.png
---

For the final mechatronics competition project, I focused on developing the robot’s control logic and navigation algorithm to maximize the number of blocks collected within the arena. I designed a zig-zag search pattern that allowed the robot to efficiently sweep through the center of the field while using QT sensors to detect and respond to the black border. The algorithm accounted for multiple edge-detection scenarios, including approaching the border at different angles, and implemented different turning behaviors to prevent the robot from driving out of bounds. I also programmed the robot to track border intersections and stop operation after completing three passes. Through this project, I worked on sensor integration, autonomous navigation, and decision-making logic for real-time robot control.

Here is a flow chart of the algorithm I made:

![Algo Flow Chart]({{ "assets/images/algo.png" | relative_url }}){: class="algo-image" width="500"}

This is what I coded up:

```c
    // "timer" : stores the value of TIMER1
    volatile float timer1 = 0;
    // "left" : stores whether left QTI sensor detects black
    volatile int left = 0;
    // "left" : stores whether right QTI sensor detects black
    volatile int right = 0;
    // "turn_orientation" : stores last turn (left = 0, right = 1)
    volatile int turn_orientation = 0;
    // "count" : number of iterations
    volatile int count = 3;

    // black intensity threshold
    #define BLACK_THRESHOLD 4900

    // Left motor pins
    int pin8 = 0; // PINB0
    int pin9 = 1; // PINB1

    // Right motor pins
    int pin5 = 5; // PIND5
    int pin6 = 6; // PIND6

    // QTI pins
    int left_qti = 3; // PIND3
    int right_qti = 2; // PIND2

    // SERVO pins
    #define SERVO1_PIN 4 // PORTB bit 4
    #define SERVO2_PIN 5 // PORTB bit 5

    // Forward declarations
    void initQTI();
    int readQTI(int pin);
    void drive_forward();
    void drive_backward();
    void drive_left();
    void drive_right();
    void stop();
    void drive_delay(int t);

    // send one PWM pulse to a servo
    void servoPulse(uint8_t pin, int pulse_us) {
        PORTB |= (1 << pin); // HIGH
        _delay_us(pulse_us); // hold for pulse width
        PORTB &= ~(1 << pin); // LOW
        _delay_us(20000 - pulse_us); // remainder of 20ms period
    }

    // send pulses for a given duration (ms) at a given pulse width
    void servoRun(uint8_t pin, int pulse_us, int duration_ms) {
    // Each cycle is 20ms, so calculate how many cycles fit in duration
    int cycles = duration_ms / 20;
        for (int i = 0; i < cycles; i++) {
            servoPulse(pin, pulse_us);
        }
    }

    // return QTI output
    int readQTI(int pin) {
        DDRD |= (1 << pin); // set as output
        PORTD |= (1 << pin); // drive HIGH
        _delay_ms(1);

        DDRD &= ~(1 << pin); // set as input
        PORTD &= ~(1 << pin); // disable pullup

        int count = 0;
        while (PIND & (1 << pin)) {
            count++;
            if (count > 5000) break;
        }
        return count;
    }

    int main(void) {
        Serial.begin(9600);
        // inits
        DDRB = 0b00110011;
        DDRD = 0b01100000;
        PORTB = 0b0;
        PORTD = 0b0;
        sei();
        // rotate servo 1 approximately 90 degrees
        #define ROTATE_MS 500 // start here and tune up/down

        int i = 0;
        int turn_orientation = 0;

        // going to the middle
        drive_forward();
        // drive the arms down
        servoRun(SERVO2_PIN, 1750, ROTATE_MS);
        servoRun(SERVO1_PIN, 2000, ROTATE_MS);
        drive_delay(2000);
        // reach the middle and turn left
        drive_left();
        drive_delay(650);
        stop();
        int initial = 0;
        while (1){
            /*
            Reads QTI sensors and determines
            whether the QTIs detect black
            */
            int leftVal = readQTI(left_qti);
            int rightVal = readQTI(right_qti);
            left = (leftVal > BLACK_THRESHOLD);
            right = (rightVal > BLACK_THRESHOLD);


            // Serial.print("L: "); Serial.print(leftVal);
            // Serial.print(" R: "); Serial.print(rightVal);
            // Serial.print(" i: "); Serial.println(i);


            // stop the robot when it detects the black border 3 times
            if (i > count){
                stop();
                break;
            }
            if (left || right){ //detected black
                stop();
                drive_backward();
                drive_delay(75);
                if (left && right){ //detect black on both QTI sensors
                // Serial.print("Detected black turn 180 ");
                drive_left();
                drive_delay(1000);
                drive_forward();
                left = 0;
                right = 0;
                } else if (left){ //detect black on left QTI sensor
                // Serial.print("Detected black turn RIGHT ");
                drive_right();
                drive_delay(750);
                stop();
                left = 0;
                } else if (right){ //detect black on right QTI sensor
                // Serial.print("Detected black turn LEFT ");
                drive_left();
                drive_delay(750);
                stop();
                right = 0;
                }
            i += 1;
            initial = 0;
            continue;
            }
            //zig zag
            if (!initial){ //adjust the turn direction
                //initial turn
                drive_right();
                drive_delay(345);
                stop();
                turn_orientation = 1;
                initial = 1;
            }
            drive_forward();
            drive_delay(500);
            if (!turn_orientation){ //last turned left
                drive_right();
                drive_delay(500);
                stop();
                turn_orientation = 1;
            } else{ //last turned right
                drive_left();
                drive_delay(500);
                stop();
                turn_orientation = 0;
            }
        }
    }

    // left QTI sensor detected black
    ISR(INT0_vect) {
        left = 1;
    }
    // right QTI sensor detected black
    ISR(INT1_vect) {
        right = 1;
    }
    // initialize QTI
    void initQTI() {
        DDRD &= ~(1 << left_qti);
        DDRD &= ~(1 << right_qti);
    }
    // drive robot backwards
    void drive_backward() {
        PORTB |= (1 << pin9);
        PORTD |= (1 << pin5);
    }
    // drive robot forwards
    void drive_forward() {
        PORTB |= (1 << pin8);
        PORTD |= (1 << pin6);
    }
    // drive robot left
    void drive_left() {
        stop();
        PORTB |= (1 << pin8);
        PORTD |= (1 << pin5);
    }
    // drive robot right
    void drive_right() {
        stop();
        PORTB |= (1 << pin9);
        PORTD |= (1 << pin6);
    }
    // stop the robot
    void stop() {
        PORTB &= ~(1 << pin8);
        PORTB &= ~(1 << pin9);
        PORTD &= ~(1 << pin5);
        PORTD &= ~(1 << pin6);
        _delay_ms(100);
    }
    //drive delay
    void drive_delay(int t) {
        _delay_ms(t);
    }
```

