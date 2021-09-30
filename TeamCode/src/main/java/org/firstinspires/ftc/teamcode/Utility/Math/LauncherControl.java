package org.firstinspires.ftc.teamcode.Utility.Math;

import org.firstinspires.ftc.teamcode.Utility.Configuration;

import java.util.List;

public class LauncherControl {


    /*
    Standard Deviation must be below a threshold to be considered steady state.
    This is necessary for launch, and also potentially for tuning the PIDF F parameter
     */
    public static boolean isSteadyState(List<Double> velocityHistory) {
        double standardDeviationOfVelocity = ListMath.standardDeviation(velocityHistory,
                Configuration.launchErrorWindowSteps);
        return standardDeviationOfVelocity < Configuration.launchStdDevErrorTicksPerSecond;
    }

    /*
    Indicates that we are at steady state and the steady state error is within acceptable limits.
     */
    public static boolean isErrorLow(List<Double> velocityHistory, double targetVelocityTicksPerSecond) {
        boolean steadyState = isSteadyState(velocityHistory);
        double velocityAverage = ListMath.average(velocityHistory,
                Configuration.launchErrorWindowSteps);
        double averageError = Math.abs( velocityAverage - targetVelocityTicksPerSecond);
        return steadyState & (averageError < Configuration.launchMeanErrorTicksPerSecond);
    }

   /*
    Error low, and firing servo is ready,
     */
    public static boolean isReadyToShoot(List<Double> velocityHistory, double targetVelocityTicksPerSecond,
                                         double fireTimestamp, double resetTimestamp, double time) {
        boolean steadyState = isSteadyState(velocityHistory);
        boolean errorLow = isErrorLow(velocityHistory, targetVelocityTicksPerSecond);
        boolean fireServoReady = fireTimestamp < resetTimestamp  &
                resetTimestamp + Configuration.LAUNCH_SERVO_DELAY < time;
        return steadyState & errorLow & fireServoReady;
    }


}
