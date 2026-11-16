/* SPDX-License-Identifier: MIT */
/**
 * @file  car_control.h
 * @brief Cascade control: wheel speed PIs inside, gyro yaw loop outside.
 *
 * Why a gyro at all: the two gearmotors never match. The same duty gives
 * different rpm, so the car pulls to one side and no amount of speed-loop
 * tuning fixes it - the speed loops are happy to run at two different speeds.
 * The yaw loop is the only thing that can see the car actually turning.
 *
 * Cascade (every CTRL_MS):
 *   outer  yaw: heading error [deg] -> differential wheel speed [rpm]
 *   inner  wheel: rpm error -> duty, one PI per wheel
 * The differential is added to the *speed targets*, never to the duty: a duty
 * offset applied after the wheel loops is cancelled by their own integrators
 * within a few control periods, which is why it looks like a no-op.
 *
 * Driving: the heading of the newest command is the reference, the yaw loop
 * holds it, so the car goes straight even with unequal friction.
 * Turning in place: one wheel coasts, the other one is driven by a yaw RATE
 * loop (deg/s), so a left/right press always spins the car at PIVOT_RATE_DPS
 * and TURN:<deg> stops on the fused angle.
 * Driving and turning at once (arc): the differential is set directly from
 * STEER_RATE_DPS and the heading loop stands aside; releasing the turn re-arms
 * the heading, so the car keeps the new direction.
 *
 * The algorithm talks to the hardware through CarControlIo, so the whole
 * controller runs on a PC with fake motors/encoders (see /tmp/ctl_sim).
 * Units: mm/s, degrees, rpm, duty -100..100.
 */
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <Arduino.h>
#include "src/app/car_tick.h"

struct CarControlIo {
    void  (*setDuty)(int l, int r) = nullptr;
    float (*rpmL)()   = nullptr;
    float (*rpmR)()   = nullptr;
    float (*yawDeg)() = nullptr;     /* unwrapped fused yaw, + = counter-clockwise */
    float (*yawRateDps)() = nullptr; /* gyro rate about the true vertical, deg/s */
    bool  (*yawValid)()   = nullptr; /* null = always usable */
    float (*mm)()     = nullptr;     /* distance travelled, for the odometer */
};

/* tuning */
#ifndef WHEEL_DIA_MM
#define WHEEL_DIA_MM     50.0f    /* wheel diameter: 157 mm per revolution */
#endif
#ifndef TURN_TRACK_MM
#define TURN_TRACK_MM    100.0f   /* distance between the two wheels, centre to centre */
#endif
#ifndef CTRL_MS
#define CTRL_MS          10       /* control period: 100 Hz, the same rate the
                                   * production boat controller runs at. 20 ms made
                                   * the yaw loop visibly lazy on the real chassis. */
#endif
#define MAX_MMPS         250.0f   /* safest top speed for the kit */
#define MAX_RPM_CMD      220.0f   /* clamp on a single wheel speed target */
#define FF_DUTY_RPM      0.70f    /* feed-forward slope, duty per rpm: ~145 rpm at
                                   * full duty is ~0.69 duty/rpm, measured on the kit. */
#define FF_DUTY_OFFSET   14.0f    /* plus the constant that beats the dead zone, so
                                   * the feed-forward alone already lands on the right
                                   * speed and the integrators only clean up */
#define KP_SPEED         1.10f    /* duty per rpm: dominated by the feed-forward
                                   * slope, so a change of the wheel-speed target
                                   * becomes a real duty change within ONE period.
                                   * 0.35 was far too soft: a 6 rpm differential
                                   * requested by the yaw loop became ~2 duty, which
                                   * the dead zone swallowed (real-car symptom: the
                                   * loop "does nothing" and the car drifts).
                                   * Higher than ~1.5 oscillates (host sim). */
#define KI_SPEED         0.60f    /* duty per rpm.s: closes the last ~10 rpm that
                                   * the dead zone leaves behind (host sim) */
#define INT_LIMIT        45.0f    /* integral clamp, duty */

/* ---- outer yaw loop: heading error -> differential wheel speed ---------------- */
#define KP_YAW           1.20f    /* rpm of differential per degree of heading error
                                   * (about 1.7 deg/s of correction per degree of
                                   * error). The production controller used 2% duty
                                   * per degree, the same order of magnitude. A weak
                                   * outer loop on a chassis with stiction never
                                   * moves, and the car just keeps drifting. */
#define KI_YAW           1.20f    /* rpm per deg.s: cancels the steady friction pull
                                   * (this is what removes the permanent offset).
                                   * 0.6 needed 5+ s to settle after a manoeuvre,
                                   * which looks exactly like "it drifts". */
#define YAW_INT_LIMIT    20.0f    /* clamp on that integral, rpm */
#define YAW_DIFF_MAX     35.0f    /* clamp on the differential, rpm (~105 deg/s) */
#define YAW_SLEW_RPM_S   80.0f    /* how fast the differential may change: still
                                   * limited so both wheels never lurch, but fast
                                   * enough to catch a real drift (8 rpm / 100 ms) */
#define KR_YAW           0.20f    /* trim on (commanded - measured) yaw rate */
#define YAW_DEAD_DEG     1.0f     /* below this the P term is ignored, so encoder
                                   * and gyro rounding cannot make the wheels
                                   * chatter. The host sim was run at 0.6 / 1.0 /
                                   * 2.0 / 3.0 deg: 0.6 and 1.0 hold the heading
                                   * to under half a degree, 2.0 wanders +-2 deg
                                   * and 3.0 fails outright. */
/* Which motor channel is the physically LEFT wheel is a wiring fact, and on this
 * chassis M1 sits on the right. It is not a runtime decision: MOTOR_SWAP_LR in
 * car_motor.h routes duty AND encoder speed together at the hardware seam
 * (car_hw.cpp), so the two inner speed loops can never end up cross-wired and
 * nothing can silently swap the arrow keys mid-session. Everything in this file
 * then talks plain "left"/"right". */

/* ---- turning: yaw RATE loop on the single driving wheel --------------------- */
#define PIVOT_RATE_DPS   60.0f    /* how fast an arrow key spins the car */
#define STEER_RATE_DPS   45.0f    /* how fast the car turns while it is also
                                   * driving (forward + left/right at the same time) */
#define PIVOT_DUTY_BASE  30       /* the wheel does not move below the dead zone */
#define PIVOT_DUTY_MIN   34       /* floor of the rate loop, duty: it has to beat
                                   * stiction on the FIRST period. With 18 the wheel
                                   * only moved once the slow integral had built up,
                                   * and the page re-sends the key every 300 ms,
                                   * which wiped that integral -> left/right
                                   * "sometimes did nothing at all". */
#define PIVOT_DUTY_OPEN  50       /* constant duty when there is no gyro at all
                                   * (the production boat span its wheels at 50%) */
#define PIVOT_DUTY_MAX   85       /* ceiling: above this the car just slips */
#define FF_PIVOT_DUTY    0.45f    /* duty per deg/s of requested rate */
#define KP_PIVOT_DUTY    0.55f    /* duty per deg/s of rate error: one duty count is
                                   * worth ~2 deg/s here, so keep the loop gain well
                                   * below 1 per period or the wheel dithers */
#define KI_PIVOT_DUTY    1.00f    /* duty per deg/s.s */
#define PIVOT_INT_LIMIT  20.0f    /* clamp on that integral, duty */
#define KP_TURN_RATE     1.60f    /* deg/s of rate command per degree of angle left */
#define TURN_RATE_MAX    90.0f    /* never ask for a faster spin than this */
#define TURN_TOL_DEG     1.5f     /* yaw criterion for TURN:<deg> */
#define TURN_TOL_MM      1.5f     /* encoder fallback criterion (no IMU) */
#define TURN_TIMEOUT_MS  6000
#define CMD_TIMEOUT_MS   1500     /* no command for this long while driving = stop */

enum CarMode : uint8_t { CAR_IDLE, CAR_SPEED, CAR_TURN, CAR_PIVOT };

class CarControl {
public:
    /** Attach hardware. `tick` may be null (millis fallback). */
    void begin(const CarControlIo &io, const CarTick *tick = nullptr);

    /** Desired linear speed, mm/s (negative = reverse). Pins the heading to hold. */
    void setSpeed(float mmps) { setDrive(mmps, 0); }

    /** The single entry point the remote uses: speed AND turn intent together.
     *  mmps == 0 with a turn = spin in place on one wheel, mmps != 0 with a turn
     *  = drive an arc. Mixing both here is what removes the "left/right sometimes
     *  does nothing" failure: before, driving and turning were two exclusive
     *  modes, and the 300 ms keep-alive of one kept killing the other. */
    void setDrive(float mmps, int turn);
    int  turnDir() const { return _turnDir; }

    /** Turn in place by `deg` (positive = counter-clockwise), closed on the gyro. */
    void turnBy(float deg);

    /** Rotate continuously: +1 = left (right wheel drives), -1 = right (left
     *  wheel drives), 0 = stop. This is what the arrow keys use: it behaves
     *  exactly like holding a key, so the page has to keep it alive. */
    void pivot(int dir);

    /** Gyro correction while driving straight. ON by default: without it the
     *  car pulls to the side of the faster wheel. */
    void holdHeading(bool on);
    bool headingHold() const { return _holdHeading; }

    /** Make the current heading the reference (turnBy and heading hold). */
    void zeroHeading();

    /** Immediate stop: coast, clears every loop state. */
    void stop();

    /** Run the loop; call every CTRL_MS. */
    void update();

    CarMode mode() const { return _mode; }
    const char *modeName() const;

    /* observability (serial STATE log + self test) */
    float yawDiffRpm() const    { return _yawDiff; }      /* applied differential */
    float yawRateCmd() const    { return _yawRateCmd; }   /* demanded, deg/s */
    float yawRateMeas() const   { return _yawRateMeas; }  /* measured, deg/s */
    float targetMmps() const    { return _targetMmps; }
    float targetYaw() const     { return _targetYaw; }
    float turnArcMm() const     { return _turnArc; }
    float turnTravelMm() const  { return _turnTravel; }
    int   dutyL() const         { return _dutyL; }
    int   dutyR() const         { return _dutyR; }

    /** rpm of differential per deg/s of yaw rate: dV = w(rad/s) * track. */
    static float rpmPerDps();

private:
    void  driveSpeed();
    void  driveTurn();
    void  drivePivot();
    void  drivePivotRate(float rateCmdDps);
    float wheelRpmFor(float mmps) const;
    static float ffDuty(float rpm);
    float yawNow() const;
    float yawRateNow();
    bool  yawUsable() const;
    float yawDiffTarget();   /* computes + applies the slew limit */
    void  applyDuty(int l, int r);
    float rpmLnow();
    float rpmRnow();

    CarControlIo _io;
    CarMode _mode = CAR_IDLE;

    float _targetMmps = 0.0f;
    int   _turnDir = 0;                /* steering while driving: +1 left, -1 right */
    int   _dutyL = 0, _dutyR = 0;

    float _iL = 0.0f, _iR = 0.0f;      /* wheel integral terms, duty */
    float _dt = (float)CTRL_MS / 1000.0f;  /* measured control period */

    float _targetYaw = 0.0f;           /* heading reference (hold / turn target) */
    bool  _holdHeading = true;         /* gyro differential is the default */
    float _iYaw = 0.0f;                /* heading integral, rpm */
    float _yawDiff = 0.0f;             /* last applied differential, rpm */
    float _yawRateCmd = 0.0f;
    float _yawRateMeas = 0.0f;
    float _prevYaw = 0.0f;             /* for the rate fallback */
    float _iPivot = 0.0f;              /* pivot rate-loop integral, duty */

    /* pivot / turn state */
    int   _pivotDir = 0;               /* +1 left, -1 right, 0 none */
    float _turnArc = 0.0f;             /* mm the driving wheel must roll (no IMU) */
    float _turnTravel = 0.0f;          /* mm the driving wheel rolled so far */

    const CarTick *_tick = nullptr;
    uint32_t _lastTick = 0;
    unsigned long _lastAt = 0;
    unsigned long _cmdAt = 0;
    unsigned long _turnAt = 0;

};

#endif /* CAR_CONTROL_H */