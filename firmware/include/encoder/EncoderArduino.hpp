#pragma once
#include "../common/config.hpp"
#include "EncoderMath.hpp"
#include "IEncoder.hpp"

class ArduinoEncoder : public IEncoder {
public:
    bool init() override;
    float getSpeedRight() override;
    float getSpeedLeft() override;

private:
};
