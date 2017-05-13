#ifndef minikame_h
#define minikame_h
#include "Octosnake.h"

// servo index to board_pins

#define FRONT_RIGHT_HIP 0
#define FRONT_LEFT_HIP 1
#define FRONT_RIGHT_LEG 2
#define FRONT_LEFT_LEG 3
#define BACK_RIGHT_HIP 4
#define BACK_LEFT_HIP 5
#define BACK_RIGHT_LEG 6
#define BACK_LEFT_LEG 7

class MiniKame {

  public:
    MiniKame();
    void init();
    void run(int dir = 1, float steps = 1, float T = 550);
    void walk(int dir = 1, float steps = 1, float T = 550);
    void omniWalk(float steps, float T, bool side, float turn_factor);
    void turnL(float steps, float period);
    void turnR(float steps, float period);
    void moonwalkL(float steps, float period);
    void dance(float steps, float period);
    void upDown(float steps, float period);
    void pushUp(float steps, float period);
    void hello();
    void jump();
    void home();
    void frontBack(float steps, float period);

    void setServo(int id, float target);
    void reverseServo(int id);
    float getServo(int id);
    void moveServos(int time, float target[8]);
    void setTrim(int index, int value) {
      trim[index] = value;
    }
    void storeTrim();
    void loadTrim();
  private:
    Oscillator oscillator[8];
    Servo servo[8];
    int board_pins[8];
    int trim[8]; //deviation servo offset
    bool reverse[8];
    unsigned long _init_time;
    unsigned long _final_time;
    unsigned long _partial_time;
    float _increment[8];
    float _servo_position[8];

    void execute(float steps, float period[8], int amplitude[8], int offset[8], int phase[8]);
    inline int angToUsec(float value) {
      return value / 180 * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) + MIN_PULSE_WIDTH;
    };
    int EEPROMReadWord(int p_address);
    void EEPROMWriteWord(int p_address, int p_value);
};

#endif
