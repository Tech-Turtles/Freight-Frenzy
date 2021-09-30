package org.firstinspires.ftc.teamcode.Utility;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Configuration {

    public static double HOPPER_OPEN_POS    = 0.32;
    public static double HOPPER_PUSH_POS    = 0.21;
    public static double LAUNCH_SERVO_DELAY = 0.3;

    public static final double deadzone = 0.49;

    public static final double LAUNCHER_THEORETICAL_MAX = 3387.096774;

    public static int WOBBLE_STORE  = 0;
    public static int WOBBLE_UP     = 977;
    public static int WOBBLE_DOWN   = 3110;

    public static double WOBBLE_GRABBED_IN = 0.5;
    public static double WOBBLE_OUT_IN = 1.5;
    public static double WOBBLE_GRABBED_MAX_IN = 2.4;

    public static double SPATULA_STORE  = 0.3;
    public static double SPATULA_DOWN   = 0.85;

    public static double highGoalSpeed  = 0.55;
    public static double powerShotSpeed = 0.48;
    public static double wobblePower    = 1.0;
    public static double intakePower    = 1.0;

    public static double launchMeanErrorTicksPerSecond = 80.0;
    public static double launchStdDevErrorTicksPerSecond = 30.0;
    public static int launchErrorWindowSteps = 18;

    public static PIDFCoefficients pidfCoeffFeedForward =  new PIDFCoefficients(40,0,0,15);
    public static PIDFCoefficients pidfCoeffOriginalIntegral =  new PIDFCoefficients(10,3,0,0);

    /*
        PowerSpeed is meant to be the speed given to the motor, from [-1,1]
        which the controller PID interprets as a objective rate in ticks per second.
     */
    public static int getLaunchTicksPerSecondFromPowerSpeed(double powerSpeed) {
        return (int) (LAUNCHER_THEORETICAL_MAX * powerSpeed);
    }
}