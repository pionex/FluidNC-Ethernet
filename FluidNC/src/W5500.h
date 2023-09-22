// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */

#include "Configuration/Configurable.h"
#include "Pin.h"
#include "Error.h"

#include <cstdint>

class W5500 : public Configuration::Configurable {
public:

private:
    bool setupW5500();
    
    Pin   _interrupt;
    Pin   _cs;

public:
    W5500();
    W5500(const W5500&) = delete;
    W5500& operator=(const W5500&) = delete;

    void afterParse() override;

    // Initializes pins.
    void init();

    // Configuration handlers.
    void group(Configuration::HandlerBase& handler) override {
        handler.item("cs_pin", _cs);
        handler.item("interrupt", _interrupt);
    }

    ~W5500();
};
