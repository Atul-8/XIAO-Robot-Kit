/* SPDX-License-Identifier: MIT */
/**
 * @file  car_encoder.cpp
 * @brief x4 quadrature decoding in ISRs + windowed speed / distance.
 */

#include "src/hw/car_encoder.h"

/* x4 table: delta = kQuad[(prevState << 2) | nextState], state = (A << 1) | B */
static const int8_t kQuad[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

/* the two channels the ISRs serve (set in begin()) */
static EncChan *s_chan[2] = {nullptr, nullptr};

static inline void decode(EncChan *c) {
    if (!c) return;
    uint8_t next = (uint8_t)((digitalRead(c->a) << 1) | digitalRead(c->b));
    c->count += kQuad[(c->state << 2) | next];
    c->state = next;
}

static void IRAM_ATTR isrL() { decode(s_chan[0]); }
static void IRAM_ATTR isrR() { decode(s_chan[1]); }

CarEncoder::CarEncoder(uint8_t lA, uint8_t lB, uint8_t rA, uint8_t rB) {
    _l.a = lA; _l.b = lB; _l.sign = ENC_L_SIGN;
    _r.a = rA; _r.b = rB; _r.sign = ENC_R_SIGN;
}

bool CarEncoder::begin(const CarTick *tick) {
    _tick = tick;
    EncChan *ch[2] = {&_l, &_r};
    for (int i = 0; i < 2; i++) {
        pinMode(ch[i]->a, INPUT_PULLUP);
        pinMode(ch[i]->b, INPUT_PULLUP);
        ch[i]->count = 0;
        ch[i]->state = (uint8_t)((digitalRead(ch[i]->a) << 1) | digitalRead(ch[i]->b));
    }
    s_chan[0] = &_l;
    s_chan[1] = &_r;
    attachInterrupt(digitalPinToInterrupt(_l.a), isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_l.b), isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_r.a), isrR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_r.b), isrR, CHANGE);

    _windowAt = millis();
    _lastTick = tick ? tick->count() : 0;
    _lastL = _lastR = 0;
    _counts = 0.0f;
    _rpmL = _rpmR = 0.0f;
    return true;
}

/* atomic 32-bit read (ISR writes it) */
static int32_t readCount(const EncChan *c) {
    noInterrupts();
    int32_t v = c->count;
    interrupts();
    return v;
}

int32_t CarEncoder::left() const  { return readCount(&_l) * _l.sign; }
int32_t CarEncoder::right() const { return readCount(&_r) * _r.sign; }

float CarEncoder::countsToRpm(int32_t delta, float dt) const {
    if (dt <= 0.0f) return 0.0f;
    return (float)delta * (60.0f / ENC_CPR) / dt;      /* counts -> rev/min */
}

void CarEncoder::update() {
    /* window timing: heartbeat when attached, millis() otherwise */
    float dt;
    if (_tick) {
        uint32_t elapsed = _tick->elapsed(_lastTick);
        dt = (float)elapsed * _tick->dt();
        if (dt * 1000.0f < (float)ENC_WINDOW_MS) return;
        _lastTick += elapsed;
    } else {
        unsigned long now = millis();
        dt = (float)(now - _windowAt) / 1000.0f;
        if (dt * 1000.0f < (float)ENC_WINDOW_MS) return;
        _windowAt = now;
    }

    int32_t l = left(), r = right();
    _rpmL = countsToRpm(l - _lastL, dt);
    _rpmR = countsToRpm(r - _lastR, dt);

    /* distance integrates the wheel pair, so one slipping wheel does not
     * double the reading */
    _counts += (float)((l - _lastL) + (r - _lastR)) * 0.5f;

    _lastL = l;
    _lastR = r;
}

float CarEncoder::mm() const {
    const float circ = 3.14159265f * _wheelDia;        /* mm per revolution */
    return _counts * circ / ENC_CPR;
}
