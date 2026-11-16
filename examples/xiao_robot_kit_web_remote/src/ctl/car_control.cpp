/* SPDX-License-Identifier: MIT */
/**
 * @file  car_control.cpp
 * @brief Gyro-closed cascade: wheel speed PIs inside, heading / yaw-rate outside.
 *
 * The problem this solves: two gearmotors never match. At the same duty the
 * car pulls to the side of the faster wheel, and the per-wheel speed loops do
 * not care - they happily hold two different speeds. Only the gyro can see it.
 *
 * Straight driving (CAR_SPEED), every CTRL_MS:
 *     err  = targetYaw - yaw                       (deg, target pinned by SPD)
 *     iYaw += KI_YAW * err * dt                    (removes the steady pull)
 *     dv   = KP_YAW * err + iYaw                   (rpm of differential)
 *     dv  += KR_YAW * (dv - measuredRate)           (trims slip / diameter error)
 *     targetL = base - dv,  targetR = base + dv     (speed targets, not duty!)
 *     per wheel: duty = FF*target + KP*(target - rpm) + I
 *
 * The differential goes into the *speed targets* on purpose. A duty offset
 * added after the wheel loops is eaten by their integrators within a few
 * periods, which is why the old "duty bump" heading hold looked like a no-op.
 *
 * Turning (CAR_PIVOT / CAR_TURN): the inner wheel coasts, only the outer one
 * drives, so the car rotates around the stopped wheel (two wheels fighting on
 * carpet just buzz). Its duty comes from a yaw RATE loop instead of a constant:
 *     rate  = PIVOT_RATE_DPS                  (arrow keys: +-1 continuous)
 *     rate  = clamp(KP_TURN_RATE * angleLeft) (TURN:<deg>: stops on the gyro)
 *     duty  = base + FF*|rate| + KP*(rate - measuredRate) + I
 * so the same key gives the same spin on carpet, on wood and with a half-empty
 * battery, and TURN:90 lands on 90 deg whatever the friction is.
 *
 * Without a usable gyro everything degrades to the old open-loop behaviour:
 * constant pivot duty and the encoder-arc stop criterion.
 */

#include "src/ctl/car_control.h"
#include <math.h>

#define PI_F 3.14159265f

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* shortest signed difference, deg */
static inline float wrap180(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

const char *CarControl::modeName() const {
    switch (_mode) {
        case CAR_SPEED: return "SPD";
        case CAR_TURN:  return "TURN";
        case CAR_PIVOT: return _pivotDir > 0 ? "LEFT" : (_pivotDir < 0 ? "RGHT" : "PIVT");
        default:        return "IDLE";
    }
}

float CarControl::rpmPerDps() {
    /* yaw rate w [rad/s] needs dv = w * track [mm/s];
     * -> rpm / (deg/s) = (pi/180) * track * 60 / (pi * dia) = track / (3 * dia) */
    return TURN_TRACK_MM / (3.0f * WHEEL_DIA_MM);
}

void CarControl::begin(const CarControlIo &io, const CarTick *tick) {
    _tick = tick;
    _io = io;
    _mode = CAR_IDLE;
    _targetMmps = 0.0f;
    _dutyL = _dutyR = 0;
    _iL = _iR = 0.0f;
    _iYaw = 0.0f;
    _iPivot = 0.0f;
    _yawDiff = _yawRateCmd = _yawRateMeas = 0.0f;
    _prevYaw = yawNow();
    _targetYaw = _prevYaw;
    _holdHeading = true;               /* the gyro differential is the point */
    _pivotDir = 0;
    _turnArc = 0.0f;
    _turnTravel = 0.0f;
    _lastAt = millis();
    _lastTick = tick ? tick->count() : 0;
    _cmdAt = _lastAt;
    _turnAt = _lastAt;
    if (_io.setDuty) _io.setDuty(0, 0);
}

float CarControl::wheelRpmFor(float mmps) const {
    const float mmPerRev = PI_F * WHEEL_DIA_MM;
    return mmps * 60.0f / mmPerRev;
}

float CarControl::yawNow() const { return _io.yawDeg ? _io.yawDeg() : 0.0f; }

bool CarControl::yawUsable() const {
    if (!_io.yawDeg) return false;
    if (_io.yawValid && !_io.yawValid()) return false;
    return true;
}

/* measured yaw rate, deg/s, + = counter-clockwise */
float CarControl::yawRateNow() {
    const float y = yawNow();
    float r = 0.0f;
    if (yawUsable()) {
        if (_io.yawRateDps) {
            r = _io.yawRateDps();
        } else if (_dt > 1e-4f) {      /* fallback: differentiate the angle */
            r = wrap180(y - _prevYaw) / _dt;
        }
    }
    _prevYaw = y;
    return r;
}

void CarControl::applyDuty(int l, int r) {
    _dutyL = l;
    _dutyR = r;
    if (_io.setDuty) _io.setDuty(l, r);
}

/* the two encoder speeds, already in the same left/right sense as the duty
 * output (car_hw.cpp applies MOTOR_SWAP_LR to both sides at once) */
float CarControl::rpmLnow() { return _io.rpmL ? _io.rpmL() : 0.0f; }
float CarControl::rpmRnow() { return _io.rpmR ? _io.rpmR() : 0.0f; }

void CarControl::setDrive(float mmps, int turn) {
    turn = (turn > 0) ? 1 : ((turn < 0) ? -1 : 0);
    const float want = clampf(mmps, -MAX_MMPS, MAX_MMPS);

    _cmdAt = millis();                 /* keep-alive counts as a live command */

    /* no forward speed: the turn intent alone means "spin on one wheel" */
    if (want == 0.0f) {
        _turnDir = 0;
        pivot(turn);
        return;
    }

    const bool same = (_mode == CAR_SPEED) && (want == _targetMmps)
                                       && (turn == _turnDir);
    if (same) return;                  /* repeated command: keep the integrals */

    _mode = CAR_SPEED;
    _targetMmps = want;
    _turnDir = turn;
    _pivotDir = 0;
    _iL = _iR = 0.0f;                  /* no wind-up from the previous command */
    _iYaw = 0.0f;
    _yawDiff = 0.0f;
    /* every new drive command re-arms the heading it starts from, so releasing
     * a turn leaves the car holding the new direction (what the production
     * controller gets by zeroing yaw on every turn) */
    if (_holdHeading) _targetYaw = yawNow();
}

void CarControl::holdHeading(bool on) {
    _holdHeading = on;
    _iYaw = 0.0f;
    if (on) _targetYaw = yawNow();
    _cmdAt = millis();
}

void CarControl::turnBy(float deg) {
    if (deg == 0.0f) return;
    _targetYaw = wrap180(yawNow() + deg);
    /* pivoting about the stopped wheel: the driving wheel travels track * angle
     * (only used when there is no usable gyro) */
    _turnArc = fabsf(deg) * PI_F / 180.0f * TURN_TRACK_MM;
    _turnTravel = 0.0f;
    _mode = CAR_TURN;
    _turnDir = 0;
    _pivotDir = (deg > 0.0f) ? 1 : -1;
    _iL = _iR = 0.0f;
    _iPivot = 0.0f;
    _cmdAt = millis();
    _turnAt = _cmdAt;
}

void CarControl::pivot(int dir) {
    dir = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
    _cmdAt = millis();
    if (dir == 0) { stop(); return; }
    /* the page re-sends the key every 300 ms: that is a keep-alive, NOT a new
     * command. Re-entering here used to reset the rate-loop integral three
     * times a second, so on a chassis whose dead zone is above the feed-forward
     * the wheel only crept forward for a few periods per key press - the
     * "left/right sometimes does nothing" report. */
    if (_mode == CAR_PIVOT && _pivotDir == dir) return;

    _turnDir = 0;
    _yawDiff = 0.0f;
    _pivotDir = dir;
    _mode = CAR_PIVOT;
    _targetMmps = 0.0f;
    _iL = _iR = 0.0f;
    _iPivot = 0.0f;
}

void CarControl::zeroHeading() {
    _targetYaw = wrap180(yawNow());
    _iYaw = 0.0f;
    _cmdAt = millis();
}

void CarControl::stop() {
    _mode = CAR_IDLE;
    _pivotDir = 0;
    _turnDir = 0;
    _targetMmps = 0.0f;
    _iL = _iR = 0.0f;
    _iYaw = 0.0f;
    _iPivot = 0.0f;
    _yawDiff = _yawRateCmd = 0.0f;
    applyDuty(0, 0);
}

void CarControl::update() {
    /* Fixed control period, timed by the heartbeat when attached. The elapsed
     * time is measured, never assumed: a shorter period than CTRL_MS would make
     * the loops integrate their own distances twice as fast as the car moves. */
    float dt;
    if (_tick) {
        uint32_t elapsed = _tick->elapsed(_lastTick);
        if (elapsed == 0) return;
        dt = (float)elapsed * _tick->dt();
        if (dt * 1000.0f + 1e-3f < (float)CTRL_MS) return;
        _lastTick += elapsed;
    } else {
        unsigned long t = millis();
        dt = (float)(t - _lastAt) / 1000.0f;
        if (dt * 1000.0f < (float)CTRL_MS) return;
        _lastAt = t;
    }
    _dt = dt;

    /* watchdog: a dropped Wi-Fi link must not leave the car driving or
     * spinning. A turn by angle is bounded by its own timeout, so it is
     * excluded. */
    unsigned long now = millis();
    if ((_mode == CAR_SPEED || _mode == CAR_PIVOT) && now - _cmdAt > CMD_TIMEOUT_MS) {
        stop();
        return;
    }

    _yawRateMeas = yawRateNow();       /* what the car is doing right now */

    if (_mode == CAR_TURN)       driveTurn();
    else if (_mode == CAR_PIVOT) drivePivot();
    else if (_mode == CAR_SPEED) driveSpeed();
    else applyDuty(0, 0);
}

/* heading error -> rpm of wheel-speed differential, + = turn left */
float CarControl::yawDiffTarget() {
    float d;

    if (_turnDir != 0) {
        /* steering while driving: an open rate demand. The heading loop is out
         * of the way, otherwise "turn" and "hold" would fight each other. */
        _iYaw = 0.0f;
        d = STEER_RATE_DPS * rpmPerDps() * (float)_turnDir;
    } else if (!_holdHeading || !yawUsable()) {
        _iYaw = 0.0f;
        _yawDiff = 0.0f;
        return 0.0f;
    } else {
        const float err = wrap180(_targetYaw - yawNow());
        float p = 0.0f;
        if (fabsf(err) > YAW_DEAD_DEG) p = KP_YAW * err;

        /* the integral is what cancels a constant pull: unequal friction, a bent
         * chassis, one wheel a hair bigger than the other */
        _iYaw = clampf(_iYaw + KI_YAW * err * _dt, -YAW_INT_LIMIT, YAW_INT_LIMIT);

        d = p + _iYaw;
        /* trim: if the car rotates less than this differential implies (slip,
         * carpet, diameter mismatch) push harder */
        const float meas = _yawRateMeas * rpmPerDps();
        d += KR_YAW * (d - meas);
    }

    d = clampf(d, -YAW_DIFF_MAX, YAW_DIFF_MAX);
    /* slew limit: the differential is a *speed target*, and a step makes both
     * wheels lurch - which kicks the gyro and the loop fights itself */
    const float step = YAW_SLEW_RPM_S * _dt;
    if (d > _yawDiff + step)      d = _yawDiff + step;
    else if (d < _yawDiff - step) d = _yawDiff - step;
    _yawDiff = d;
    return d;
}

/* duty that alone gets close to `rpm`, dead zone included */
float CarControl::ffDuty(float rpm) {
    float d = FF_DUTY_RPM * rpm;
    if (rpm > 0.5f)       d += FF_DUTY_OFFSET;
    else if (rpm < -0.5f) d -= FF_DUTY_OFFSET;
    return d;
}

void CarControl::driveSpeed() {
    const float base = clampf(wheelRpmFor(_targetMmps), -MAX_RPM_CMD, MAX_RPM_CMD);
    float dv = yawDiffTarget();

    /* The production boat controller only ever SLOWS the fast wheel
     * (abs(base - mix)) and never lets the correction reverse a motor.
     * Same rule here, with 10 % of base kept as margin: at 150 mm/s this
     * changes nothing, but at 30 mm/s it stops the heading loop from asking
     * for a wheel to spin backwards - a reversed wheel has to cross the dead
     * zone again, so the car would lurch and the gyro would see a spike. */
    const float dvMax = fminf(YAW_DIFF_MAX, fabsf(base) * 0.9f);
    dv = clampf(dv, -dvMax, dvMax);

    /* one wheel faster than the other: this is the differential drive turn */
    const float tL = clampf(base - dv, -MAX_RPM_CMD, MAX_RPM_CMD);
    const float tR = clampf(base + dv, -MAX_RPM_CMD, MAX_RPM_CMD);

    const float rpmL = rpmLnow();
    const float rpmR = rpmRnow();
    const float errL = tL - rpmL;
    const float errR = tR - rpmR;

    _iL = clampf(_iL + KI_SPEED * errL * _dt, -INT_LIMIT, INT_LIMIT);
    _iR = clampf(_iR + KI_SPEED * errR * _dt, -INT_LIMIT, INT_LIMIT);

    /* feed-forward = slope + the constant that beats the dead zone, so an
     * unequal pair of motors gets an almost correct duty on the first period */
    const float dutyL = ffDuty(tL) + KP_SPEED * errL + _iL;
    const float dutyR = ffDuty(tR) + KP_SPEED * errR + _iR;

    _yawRateCmd = dv / rpmPerDps();

    applyDuty((int)lroundf(clampf(dutyL, -100.0f, 100.0f)),
              (int)lroundf(clampf(dutyR, -100.0f, 100.0f)));
}

/* single-wheel pivot with a yaw RATE loop: rateCmdDps > 0 means "turn left" */
void CarControl::drivePivotRate(float rateCmdDps) {
    _yawRateCmd = rateCmdDps;
    const int dir = (rateCmdDps > 0.0f) ? 1 : ((rateCmdDps < 0.0f) ? -1 : 0);
    if (dir == 0) { applyDuty(0, 0); return; }

    if (dir != _pivotDir) {            /* the car has to rotate the other way */
        _pivotDir = dir;
        _iPivot = 0.0f;
    }

    /* positive = under-rotating in the demanded direction */
    const float rateErr = (rateCmdDps - _yawRateMeas) * (float)dir;
    _iPivot = clampf(_iPivot + KI_PIVOT_DUTY * rateErr * _dt,
                     -PIVOT_INT_LIMIT, PIVOT_INT_LIMIT);

    float duty = (float)PIVOT_DUTY_BASE
               + FF_PIVOT_DUTY * fabsf(rateCmdDps)
               + KP_PIVOT_DUTY * rateErr
               + _iPivot;
    duty = clampf(duty, (float)PIVOT_DUTY_MIN, (float)PIVOT_DUTY_MAX);

    const int d = (int)lroundf(duty);
    const int ds = dir;
    if (ds > 0) applyDuty(0, d);       /* right wheel forward: turn left  */
    else        applyDuty(d, 0);       /* left wheel forward:  turn right */
}

void CarControl::drivePivot() {
    if (yawUsable()) {
        drivePivotRate(PIVOT_RATE_DPS * (float)_pivotDir);
        return;
    }
    /* no gyro: the old open-loop pivot still behaves like a held key */
    _yawRateCmd = 0.0f;
    const int d = _pivotDir;
    if (d > 0)      applyDuty(0, PIVOT_DUTY_OPEN);
    else if (d < 0) applyDuty(PIVOT_DUTY_OPEN, 0);
    else            applyDuty(0, 0);
}

void CarControl::driveTurn() {
    /* gyro path: angle error -> yaw rate -> outer wheel duty, stop on the angle */
    if (yawUsable()) {
        const float err = wrap180(_targetYaw - yawNow());
        if (fabsf(err) <= TURN_TOL_DEG || millis() - _turnAt > TURN_TIMEOUT_MS) {
            stop();
            return;
        }
        drivePivotRate(clampf(KP_TURN_RATE * err, -TURN_RATE_MAX, TURN_RATE_MAX));
        return;
    }

    /* no gyro: drive the wheel open loop and stop on its arc length */
    _yawRateCmd = 0.0f;
    const int s = _pivotDir;
    const float rpm = (s > 0) ? rpmRnow() : rpmLnow();
    _turnTravel += fabsf(rpm) * PI_F * WHEEL_DIA_MM / 60.0f * _dt;

    if (_turnTravel >= _turnArc - TURN_TOL_MM || millis() - _turnAt > TURN_TIMEOUT_MS) {
        stop();
        return;
    }
    if (s > 0) applyDuty(0, PIVOT_DUTY_OPEN);
    else       applyDuty(PIVOT_DUTY_OPEN, 0);
}