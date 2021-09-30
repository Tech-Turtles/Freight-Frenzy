package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum Webcam {

    WEBCAM_1("webcam");

    final String name;

    Webcam(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
