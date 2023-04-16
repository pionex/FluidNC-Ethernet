// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    This is a base class for servo-type motors - ones that autonomously
    move to a specified position, instead of being moved incrementally
    by stepping.  Specific kinds of servo motors inherit from it.
*/

#include "MotorDriver.h"

namespace MotorDrivers {
    class Servo : public MotorDriver {
    public:
        int _timer_ms = 75;

        Servo();

        virtual void update() = 0;  // This must be implemented by derived classes
        void         group(Configuration::HandlerBase& handler) override { handler.item("timer_ms", _timer_ms); }

    protected:
        static void update_servo(TimerHandle_t object);
        static void schedule_update(Servo* object, int interval);
    };
}
