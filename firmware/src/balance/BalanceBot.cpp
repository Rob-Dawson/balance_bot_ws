#include "balance/BalanceBot.hpp"
#include "imu/IMUState.hpp"
#include "telemetry/Telemetry.hpp"

// #include <cstddef>
namespace {
constexpr float microToSeconds{0.000001f};
constexpr float RadToDeg{180.0F / float(PI)};

float convertToDeg(float rad) { return rad * RadToDeg; };
const char *stateToString(IMUState state) {
    switch (state) {
    case IMUState::CALIBRATING_GYRO:
        return "CALIBRATING_GYRO";
    case IMUState::CALIBRATING_ZERO:
        return "CALIBRATING_ZERO";
    case IMUState::RUNNING:
        return "RUNNING";
    default:
        return "UNKNOWN";
    }
}
} // namespace

BalanceBot::BalanceBot(IIMU &imu, ArduinoEncoder *encoder, Telemetry *telemetry)
    : imu(imu), encoder(encoder), telemetry(telemetry) {}

void BalanceBot::telemetryState() {
    telemetry->rawPitch = convertToDeg(imu.getRawPitch());
    telemetry->pitchError = convertToDeg(pitchController.getError());
    telemetry->targetPitch = convertToDeg(pitchController.getTargetPitch());
    telemetry->pitchEstimate = convertToDeg(imu.getPitch());
    telemetry->pitchRateEstimate = convertToDeg(imu.getPitchRate());

    telemetry->outputKp = pitchController.getOutputKp();
    telemetry->outputKd = pitchController.getOutputKd();
    telemetry->commandOutput = controlOutput;
    telemetry->requestedPWMLeft = motor.getRequestedPWMLeft();
    telemetry->requestedPWMRight = motor.getRequestedPWMRight();
    telemetry->appliedPWMLeft = motor.getAppliedPWMLeft();
    telemetry->appliedPWMRight = motor.getAppliedPWMRight();
    telemetry->speedLeft = encoder->getSpeedLeft();
    telemetry->speedRight = encoder->getSpeedRight();
}

void BalanceBot::telemetryPrint() {
    if (initPrint) {
        Serial.println("RawPitch,PitchError,TargetPitch,PitchEstimate,"
                       "PitchRateEstimate,OutputKp,OutputKd,"
                       "CommandOutput,RequestedPWMLeft,RequestedPWMRight,"
                       "AppliedPWMLeft,AppliedPWMRight,SpeedLeft,SpeedRight");
    }
    Serial.print(telemetry->rawPitch);
    Serial.print(",");
    Serial.print(telemetry->pitchError);
    Serial.print(",");
    Serial.print(telemetry->targetPitch);
    Serial.print(",");
    Serial.print(telemetry->pitchEstimate);
    Serial.print(",");
    Serial.print(telemetry->pitchRateEstimate);
    Serial.print(",");
    Serial.print(telemetry->outputKp);
    Serial.print(",");
    Serial.print(telemetry->outputKd);
    Serial.print(",");
    Serial.print(telemetry->commandOutput);
    Serial.print(",");
    Serial.print(telemetry->requestedPWMLeft);
    Serial.print(",");
    Serial.print(telemetry->requestedPWMRight);
    Serial.print(",");
    Serial.print(telemetry->appliedPWMLeft);
    Serial.print(",");
    Serial.print(telemetry->appliedPWMRight);
    Serial.print(",");
    Serial.print(telemetry->speedLeft);
    Serial.print(",");
    Serial.println(telemetry->speedRight);

    initPrint = false;
}

bool BalanceBot::init() {
    initHelper(imu, IMU);
    initHelper(motor, MOTOR);
    initHelperOptional(encoder, ENCODER);
    // initHelperOptional(telemetry, TELEMETRY);

    return systemStates[IMU] == ComponentStatus::READY &&
           systemStates[MOTOR] == ComponentStatus::READY;
}

void BalanceBot::onEnterState(IMUState state) {
    Serial.println(stateToString(state));
}

void BalanceBot::handleSerial() {
    if (Serial.available() == 0) {
        return;
    }

    int cmd = Serial.read();
    if (cmd == 'p') {
        float value = Serial.parseFloat();
        pitchController.setKp(value);
    } else if (cmd == 'd') {
        float value = Serial.parseFloat();
        pitchController.setKd(value);
    }
}

void BalanceBot::closedLoop(float motorA, float motorB) {
    motor.setLeftEffort(motorA);
    motor.setRightEffort(motorB);
    motor.moveLeft();
    motor.moveRight();
}

void BalanceBot::openLoop(int motorA, int motorB) {
    motor.setLeftPWM(motorA);
    motor.setRightPWM(motorB);
    motor.moveLeft();
    motor.moveRight();
}

void BalanceBot::updateIMU() {
    imu.update();
    IMUState state = imu.getState();
    if (state != previousState) {
        onEnterState(state);
        previousState = state;
    }
}
void BalanceBot::updateTelemetry() {
    if (millis() - lastPrintTime >= printTimeThreshold) {
        telemetryState();
        telemetryPrint();
        lastPrintTime = millis();
    }
}

float BalanceBot::updateVelocityControl() {
    float pitchSetpoint = velocityController.update(
        encoder->getSpeedLeft(), encoder->getSpeedRight(), m_wheelRad,
        m_desiredSpeed, dtElapsed);
    return pitchSetpoint;
}

void BalanceBot::update() {

    handleSerial();
    if (!canBalance()) {
        if (hasTelemetry()) {
            updateTelemetry();
        }
        return;
    }
    updateIMU();
    if (imu.getState() != IMUState::RUNNING) {
        return;
    }
    float pitchSetpoint = 0.0f;
    uint32_t dt = micros() - m_lastUpdateTime;
    dtElapsed = static_cast<float>(dt) * microToSeconds;

    if (canControlVelocity()) {
        pitchSetpoint = updateVelocityControl();
    }

    controlOutput = pitchController.update(pitchSetpoint, imu.getPitch(),
                                           imu.getPitchRate());
    closedLoop(controlOutput, controlOutput);
    if (hasTelemetry()) {
        updateTelemetry();
    }
    m_lastUpdateTime = micros();
}
