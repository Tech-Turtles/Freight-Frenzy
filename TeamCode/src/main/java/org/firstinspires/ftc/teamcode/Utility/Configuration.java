package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Configuration {

    public static final double deadzone = 0.49;
    public static final double ROBOT_LENGTH = 17.0;
    public static final double ROBOT_WIDTH = 17.25;
    public static int ARM_DRIVE_POS = -300;
    public static int ARM_PICKUP_POS = 0;
    public static double cargoClose = 0.3;
    public static double cargoOpen = 0.8;
    public static double intakeExtend = 0.3;
    public static double intakeRetract = 0;

}