package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum RevTouchSensors {
    WOBBLE_TOUCH("wobbleTouch");

    private final String configName;

    RevTouchSensors(String configName) {
        this.configName = configName;
    }

    public String getConfigName() {
        return configName;
    }
}
