/* SPDX-License-Identifier: MIT */
/**
 * @file  car_web.h
 * @brief HTTP page + WebSocket server for the car (single TCP port).
 *
 * The page is served on "/" and upgraded to a WebSocket on the same port, so a
 * phone browser needs nothing but the URL (http://xiaorobot.local or the IP).
 *
 * Commands arrive as text frames and are handed to the callback; telemetry is
 * pushed with send() at whatever rate the sketch chooses.
 */
#ifndef CAR_WEB_H
#define CAR_WEB_H

#include <Arduino.h>

typedef void (*CarWebCmdFn)(const char *cmd);

class CarWeb {
public:
    /** Connect to the AP and start HTTP+WS on `port`.
     *  Returns as soon as the radio is asked to join - it never waits for the
     *  link. Watch wifiUp()/the serial log for the outcome. */
    bool begin(const char *ssid, const char *pass, const char *hostname,
               uint16_t port = 80);

    /** Accept clients, parse frames, dispatch commands and keep Wi-Fi alive. */
    void poll();

    /** Send one text frame to the active controller (no-op if none). */
    void send(const char *msg);

    bool connected();
    bool wifiUp();

    /** Diagnostics for the serial `i` key: counts prove where a request stops. */
    uint32_t requests();
    uint32_t pagesServed();
    uint32_t pagesFailed();  /* dropped before the last byte: the browser saw nothing */
    int      parked();       /* sockets waiting for their header */
    int      pageOuts();     /* pages being pushed right now */
    int      lastCmdAgeMs(); /* -1 = not one command yet */

    void onCommand(CarWebCmdFn fn) { _fn = fn; }

private:
    CarWebCmdFn _fn = nullptr;
};

#endif /* CAR_WEB_H */
