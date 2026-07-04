#include "BalanceBot.hpp"

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

void BalanceBot::telemetryState() {
    telemetry.rawPitch = convertToDeg(imu.getRawPitch());

    telemetry.pitchError = convertToDeg(pitchController.getError());
    telemetry.targetPitch = convertToDeg(pitchController.getTargetPitch());
    telemetry.pitchEstimate = convertToDeg(imu.getPitch());
    telemetry.pitchRateEstimate = convertToDeg(imu.getPitchRate());

    telemetry.outputKp = pitchController.getOutputKp();
    telemetry.outputKd = pitchController.getOutputKd();
    telemetry.commandOutput = controlOutput;
    telemetry.requestedPWMLeft = motor.getRequestedPWMLeft();
    telemetry.requestedPWMRight = motor.getRequestedPWMRight();
    telemetry.appliedPWMLeft = motor.getAppliedPWMLeft();
    telemetry.appliedPWMRight = motor.getAppliedPWMRight();
    telemetry.speedLeft = encoder.getSpeedLeft();
    telemetry.speedRight = encoder.getSpeedRight();
}

void BalanceBot::telemetryPrint() {
    if (initPrint) {
        Serial.println("RawPitch,PitchError,TargetPitch,PitchEstimate,"
                       "PitchRateEstimate,OutputKp,OutputKd,"
                       "CommandOutput,RequestedPWMLeft,RequestedPWMRight,"
                       "AppliedPWMLeft,AppliedPWMRight,SpeedLeft,SpeedRight");
    }
    Serial.print(telemetry.rawPitch);
    Serial.print(",");
    Serial.print(telemetry.pitchError);
    Serial.print(",");
    Serial.print(telemetry.targetPitch);
    Serial.print(",");
    Serial.print(telemetry.pitchEstimate);
    Serial.print(",");
    Serial.print(telemetry.pitchRateEstimate);
    Serial.print(",");
    Serial.print(telemetry.outputKp);
    Serial.print(",");
    Serial.print(telemetry.outputKd);
    Serial.print(",");
    Serial.print(telemetry.commandOutput);
    Serial.print(",");
    Serial.print(telemetry.requestedPWMLeft);
    Serial.print(",");
    Serial.print(telemetry.requestedPWMRight);
    Serial.print(",");
    Serial.print(telemetry.appliedPWMLeft);
    Serial.print(",");
    Serial.print(telemetry.appliedPWMRight);
    Serial.print(",");
    Serial.print(telemetry.speedLeft);
    Serial.print(",");
    Serial.println(telemetry.speedRight);

    initPrint = false;
}

void BalanceBot::init() {
    imu.imuInit();
    encoder.init();
    motor.init();
}

void BalanceBot::onEnterState(IMUState state) {
    Serial.println(stateToString(state));
}

void BalanceBot::handleSerial() {
    if (!Serial.available())
        return;

    char cmd = Serial.read();
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

void BalanceBot::update() {

    handleSerial();
    imu.update();
    IMUState state = imu.getState();
    if (state != previousState) {
        onEnterState(state);
        previousState = state;
    }
    if (state != IMUState::RUNNING) {
        return;
    }
    uint32_t dt = micros() - m_lastUpdateTime;
    dtElapsed = dt * microToSeconds;
    float pitchSetpoint = velocityController.update(
        encoder.getSpeedLeft(), encoder.getSpeedRight(), m_wheelRad,
        m_desiredSpeed, dt);
    controlOutput = pitchController.update(pitchSetpoint, imu.getPitch(),
                                           imu.getPitchRate());
    closedLoop(controlOutput, controlOutput);

    if (millis() - lastPrintTime >= 50) {
        telemetryState();
        telemetryPrint();
        lastPrintTime = millis();
    }
    m_lastUpdateTime = micros();
}