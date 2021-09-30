package org.firstinspires.ftc.teamcode.HardwareTypes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum ContinuousServo {

    WOBBLE_LEFT("wobble_left", DcMotorSimple.Direction.FORWARD, ServoTypes.LINKED),
    WOBBLE_RIGHT("wobble_right", DcMotorSimple.Direction.REVERSE, ServoTypes.LINKED);

    private final String configName;
    private final DcMotorSimple.Direction direction;
    private final ServoTypes type;

    ContinuousServo(String configName, DcMotorSimple.Direction direction, ServoTypes type) {
        this.configName = configName;
        this.direction = direction;
        this.type = type;
    }

    public String getConfigName() {
        return configName;
    }

    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    public ServoTypes getType() {
        return type;
    }
}
