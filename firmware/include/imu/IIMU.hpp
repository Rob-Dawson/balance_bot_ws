#pragma once
#include "IMUState.hpp"

class IIMU {
public:
    virtual ~IIMU() = default;
    virtual bool init() = 0;
    virtual void update() = 0;
    virtual IMUState getState() const = 0;
    virtual float getPitch() const = 0;
    virtual float getPitchRate() const = 0;
    virtual float getRawPitch() const = 0;
};