package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.Servo;

public enum Servos {

    CARGO_GATE("cargo gate", Servo.Direction.FORWARD, ServoTypes.INDEPENDENT),
    INTAKE("intake throw", Servo.Direction.REVERSE, ServoTypes.INDEPENDENT);

    private final String configName;
    private final Servo.Direction direction;
    private final ServoTypes type;

    Servos(String configName, Servo.Direction direction, ServoTypes type) {
        this.configName = configName;
        this.direction = direction;
        this.type = type;
    }

    public String getConfigName() {
        return configName;
    }

    public Servo.Direction getDirection() {
        return direction;
    }

    public ServoTypes getType() {
        return type;
    }
}
