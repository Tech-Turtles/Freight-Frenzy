package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum ColorSensor {
    WOBBLE_SENSOR("colorSensor");

    String name;

    ColorSensor(String name) {
        this.name = name;
    }

    public String getConfig() {
        return name;
    }
}
