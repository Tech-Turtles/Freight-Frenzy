package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Math.LauncherControl;
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext.servoDelay;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.*;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double drivespeed = 1.0;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static double manualWobblePower = 0.5;
    public static double launcherSpeedOffset = 0.0;
    public static double spatulaPosition = SPATULA_STORE;
    private double currentLauncherSpeed = 0;

    private WobbleStates wobbleState = WobbleStates.MANUAL;
    private boolean wobbleArrived = false;
    private final Executive.StateMachine<Manual> stateMachine;
    private TrajectoryRR trajectoryRR;

    private Pose2d saveLocation = new Pose2d();

    // Align to Point code
    enum DriveMode {
        NORMAL_ROBOT_CENTRIC,
        NORMAL_FIELD_CENTRIC,
        ALIGN_TO_POINT,
        ALIGN_FORWARD,
        ALIGN_DIAGONAL
    }
    private DriveMode currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
    private final PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d targetPosition = new Vector2d(12*6,-12);


    private enum WobbleStates {
        MANUAL,
        UP,
        DOWN,
        STORE
    }

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        motorUtility.setPIDFCoefficientsCompensated(Motors.LAUNCHER, DcMotor.RunMode.RUN_USING_ENCODER,  Configuration.pidfCoeffFeedForward);
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.changeState(LAUNCHER, new LaunchArm_Manual());
        stateMachine.init();
        headingController.setInputBounds(-Math.PI, Math.PI);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        trajectoryRR = new TrajectoryRR(this.mecanumDrive);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive.setPoseEstimate(lastPosition == null ? new TrajectoryRR(mecanumDrive).getSTART_CENTER() : lastPosition);
//
//        leftRange = hardwareMap.get(DistanceSensor.class, "leftDistance");
//        backRange = hardwareMap.get(DistanceSensor.class, "backDistance");
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
        mecanumDrive.update(packet);
        chordedControls(); // Controls for when left bumper is held down.
        displayTelemetry();
    }

    /*
    Manual Control States
     */
    static class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.drivetrainAdvancedControls();
            opMode.drivetrainUtilityControls();
            opMode.intakeToggleControls();
        }
    }

    static class LaunchArm_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.armControls();
            opMode.launcherControls();
        }
    }

    /*
    End of Manual Control States
     */

    void drivetrainStandardControls() {
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * linearSpeed * precisionMode,
                        -gamepad1.left_stick_x * lateralSpeed * precisionMode,
                        -gamepad1.right_stick_x * rotationSpeed * precisionMode
                )
        );
    }

    void drivetrainAdvancedControls() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Pose2d driveDirection = new Pose2d();

        Vector2d fieldFrameInput = new Vector2d(
                -gamepad1.left_stick_y * linearSpeed * precisionMode,
                -gamepad1.left_stick_x * lateralSpeed * precisionMode);

       Vector2d robotFrameInput = fieldFrameInput
               .rotated(-poseEstimate.getHeading() + Math.toRadians(90.0));


        Vector2d difference;
        double theta;
        double thetaFF;
        double headingInput;
       switch (currentDriveMode) {
           case NORMAL_ROBOT_CENTRIC:
              driveDirection = new Pose2d(
                       -gamepad1.left_stick_y * linearSpeed * precisionMode,
                       -gamepad1.left_stick_x * lateralSpeed * precisionMode,
                       -gamepad1.right_stick_x * rotationSpeed * precisionMode
               );
                break;
           case NORMAL_FIELD_CENTRIC:
               driveDirection = new Pose2d(
                       robotFrameInput.getX(), robotFrameInput.getY(),
                       -gamepad1.right_stick_x * rotationSpeed * precisionMode
               );
               break;
           case ALIGN_TO_POINT:
               // Difference between the target vector and the bot's position
               difference = targetPosition.minus(poseEstimate.vec());
               // Obtain the target angle for feedback and derivative for feedforward
               theta = difference.angle() + Math.toRadians(180.0);

               // Not technically omega because its power. This is the derivative of atan2
               thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

               // Set the target heading for the heading controller to our desired angle
               headingController.setTargetPosition(theta);

               // Set desired angular velocity to the heading controller output + angular
               // velocity feedforward
               headingInput = (headingController.update(poseEstimate.getHeading())
                       * DriveConstants.kV + thetaFF)
                       * DriveConstants.TRACK_WIDTH;

               // Combine the field centric x/y velocity with our derived angular velocity
               driveDirection = new Pose2d( robotFrameInput, headingInput );
               break;
           case ALIGN_FORWARD:
               theta = Math.toRadians(180.0);
               thetaFF = 0;
               // Set the target heading for the heading controller to our desired angle
               headingController.setTargetPosition(theta);

               // Set desired angular velocity to the heading controller output + angular
               // velocity feedforward
               headingInput = (headingController.update(poseEstimate.getHeading())
                       * DriveConstants.kV + thetaFF)
                       * DriveConstants.TRACK_WIDTH;

               // Combine the field centric x/y velocity with our derived angular velocity
               driveDirection = new Pose2d( robotFrameInput, headingInput );
               break;
           case ALIGN_DIAGONAL:
               theta = Math.toRadians(157.0);
               thetaFF = 0;
               // Set the target heading for the heading controller to our desired angle
               headingController.setTargetPosition(theta);

               // Set desired angular velocity to the heading controller output + angular
               // velocity feedforward
               headingInput = (headingController.update(poseEstimate.getHeading())
                       * DriveConstants.kV + thetaFF)
                       * DriveConstants.TRACK_WIDTH;

               // Combine the field centric x/y velocity with our derived angular velocity
               driveDirection = new Pose2d( robotFrameInput, headingInput );
       }
       // If any heading input is given, revert to robot centric.
        if(currentDriveMode != DriveMode.NORMAL_FIELD_CENTRIC && Math.abs(primary.right_stick_x) > 0.1) {
            currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
        }
        mecanumDrive.setWeightedDrivePower( driveDirection );
    }

    void drivetrainUtilityControls() {
        if(!primary.leftBumper()) {
//            if (primary.YOnce()) {
//                mecanumDrive.clearEstimatedPose();
//            }

            if (primary.rightBumperOnce()) {
                rotationSpeed = precisionMode == 1.0 ? 0.75 : 1.0;
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
            }

            if(primary.rightStickButtonOnce()) {
                currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
                stateMachine.changeState(DRIVE, new Drive_Manual_AllStates());
            } else if(primary.dpadUpOnce()) {
                currentDriveMode = DriveMode.ALIGN_DIAGONAL;
                stateMachine.changeState(DRIVE, new Drive_Manual_AllStates());
            } else if(primary.dpadDownOnce()) {
                currentDriveMode = DriveMode.ALIGN_FORWARD;
                stateMachine.changeState(DRIVE, new Drive_Manual_AllStates());
            }
        }
    }

    boolean autoRunForward = false;
    void intakeToggleControls() {
        if (primary.rightTriggerOnce()) {
            autoRunForward = true;
        }

        if(primary.left_trigger > deadzone) {
            autoRunForward = false;
            motorUtility.setPower(Motors.INTAKE, -intakePower);
        } else {
            motorUtility.setPower(Motors.INTAKE, autoRunForward ? intakePower : 0);
        }


        if(primary.XOnce()) {
            spatulaPosition = SPATULA_DOWN;
        } else if(primary.BOnce()) {
            spatulaPosition = SPATULA_STORE;
        }

        servoUtility.setAngle(Servos.SPATULA, spatulaPosition);
    }

    void intakeControls() {
        if (primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, intakePower);
        }
        else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -intakePower);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0.0);
        }
    }


    void armControls() {
        wobbleState = wobbleArrived || Math.abs(secondary.right_stick_y) > deadzone ? WobbleStates.MANUAL : wobbleState;

        if (secondary.XOnce())
            wobbleState = WobbleStates.STORE;
        else if (secondary.BOnce() && !secondary.start())
            wobbleState = WobbleStates.DOWN;
        else if (secondary.YOnce())
            wobbleState = WobbleStates.UP;

        switch (wobbleState) {
            case UP:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_UP, wobblePower);
                break;
            case DOWN:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_DOWN, wobblePower);
                break;
            case STORE:
                wobbleArrived = motorUtility.goToPosition(Motors.WOBBLE_ARM, WOBBLE_STORE, wobblePower);
                break;
            case MANUAL:
                motorUtility.setPower(Motors.WOBBLE_ARM, -secondary.right_stick_y * manualWobblePower);
        }

        servoUtility.setPower(ContinuousServo.WOBBLE_LEFT, secondary.left_stick_y);
        servoUtility.setPower(ContinuousServo.WOBBLE_RIGHT, secondary.left_stick_y);
    }

    void launcherControls() {
        if (secondary.dpadUpOnce())
            launcherSpeedOffset = Math.min(launcherSpeedOffset + 0.01, 1.0);
        else if (secondary.dpadDownOnce())
            launcherSpeedOffset = Math.max(launcherSpeedOffset - 0.01, 0);

        if (secondary.right_trigger > deadzone) {
            currentLauncherSpeed = Math.min(highGoalSpeed + launcherSpeedOffset, 1.0);
        } else if (secondary.left_trigger > deadzone) {
            currentLauncherSpeed = Math.min(powerShotSpeed + launcherSpeedOffset, 1.0);
        } else {
            currentLauncherSpeed = 0;
        }

        motorUtility.setPower(Motors.LAUNCHER, currentLauncherSpeed);

        if (secondary.rightBumper())
            servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
        else if( !secondary.leftBumper())
            servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);

        // Smart shoot test
        if (secondary.leftBumper()) {
            // Looks like you just shot, wait a bit
            if(LauncherControl.isErrorLow(launchVelocityHistory,
                    currentLauncherSpeed * LAUNCHER_THEORETICAL_MAX)) {
                servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
            } else {
                servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
            }
        }
    }


    void chordedControls() {
        if(primary.leftBumper()) {

//            if(primary.dpadDownOnce())  // Drive to auto start position
//                stateMachine.changeState(DRIVE, new Drive_ToPose(trajectoryRR.getSTART_CENTER()));
//
//            if (primary.AOnce()) // Set pose to parked center (auto start position)
//                mecanumDrive.setPoseEstimate(trajectoryRR.getSTART_CENTER());
//
//            if (primary.dpadRightOnce()) // Goto Save Location
//                stateMachine.changeState(DRIVE, new Drive_ToPose(saveLocation));
//
//            if (primary.BOnce()) // Save Location
//                saveLocation = mecanumDrive.getPoseEstimate();
//
//            if (primary.dpadUpOnce())
//                stateMachine.changeState(DRIVE, new Drive_ToPose(trajectoryRR.getSHOOT_HIGHGOAL()));

            if(primary.YOnce()) { // set pose to nearest corner
                mecanumDrive.setPoseEstimate(
                        TrajectoryRR.getNearestCornerPose2d(
                                mecanumDrive.getPoseEstimate())
                );
            }

            if(primary.BOnce()) {
                // Do powershots
                stateMachine.changeState(DRIVE,
//                        new Drive_ToPose(new Pose2d(trajectoryRR.getPOWERSHOT_LEFT().getX(), -59.0, Math.toRadians(180.0)),
                            new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_RIGHT(),
                                new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_CENTER(),
                                        new Drive_moveAndShoot(trajectoryRR.getPOWERSHOT_LEFT(),
                                                new Drive_Manual_AllStates()))));
            }

//            if(primary.XOnce()) {
//                // Drive test to show angle error in start position
//                stateMachine.changeState(DRIVE,
//                        new Drive_ToPose(trajectoryRR.getSTART_CENTER()
//                        .plus(new Pose2d(96,0,0))));
//            }
        }
    }

    boolean isDrivetrainManualInputActive() {
        double threshold = 0.3;
        return (Math.abs(primary.left_stick_x) > threshold)
            || (Math.abs(primary.left_stick_y) > threshold)
            || (Math.abs(primary.right_stick_x) > threshold);
    }

    void displayTelemetry() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("Launcher speed Offset:", df.format(launcherSpeedOffset));
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(packet != null) {
            packet.put("Precision mode:      ", df.format(precisionMode));
            packet.put("Launcher speed:      ", df.format(currentLauncherSpeed));
            packet.put("Launch velocity:     ", motorUtility.getVelocity(Motors.LAUNCHER));
            packet.put("Launch target velocity:",currentLauncherSpeed * LAUNCHER_THEORETICAL_MAX);
            packet.put("Hopper position:     ", servoUtility.getAngle(Servos.HOPPER));
//            packet.put("leftDistance range   ", df_precise.format(leftRange.getDistance(DistanceUnit.INCH)));
//            packet.put("backDistance range   ", df_precise.format(backRange.getDistance(DistanceUnit.INCH)));
            try {
                packet.put("Wobble Distance: ", df.format(getDistance(getColorSensor(ColorSensor.WOBBLE_SENSOR))));
            } catch (NullPointerException ignore) {}
            packet.put("Drive speed:         ", df.format(drivespeed));
            packet.put("Precision speed:     ", df.format(precisionPercentage));
            packet.put("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        }
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }

    /*
    Auto Control States
     */
    class Drive_ToPose extends Executive.StateBase<Manual> {
        Trajectory trajectory;
        Pose2d final_pose;
        Executive.StateBase<Manual> nextDriveState;

        Drive_ToPose(Pose2d final_pose) {
            this.final_pose = final_pose;
            this.nextDriveState = new Drive_Manual_AllStates();
        }

        Drive_ToPose(Pose2d final_pose, Executive.StateBase<Manual> nextDriveState) {
            this.final_pose = final_pose;
            this.nextDriveState = nextDriveState;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(final_pose)
                    .build();
            mecanumDrive.followTrajectoryAsync(trajectory);
        }

        @Override
        public void update() {
            super.update();
            if(mecanumDrive.isIdle() || isDrivetrainManualInputActive()) {
                stopAutoDriving(); // In case still mid trajectory
                nextState(DRIVE, nextDriveState);
            }
        }
    }

    class Drive_moveAndShoot extends Executive.StateBase<Manual> {
        boolean arrived = false;
        Trajectory trajectory;
        Pose2d final_pose;
        Executive.StateBase<Manual> nextDriveState;

        Drive_moveAndShoot(Pose2d final_pose, Executive.StateBase<Manual> nextDriveState) {
            this.final_pose = final_pose;
            this.nextDriveState = nextDriveState;
        }

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(final_pose)
                    .build();
            mecanumDrive.followTrajectoryAsync(trajectory);
            nextState(LAUNCHER, new Launch_windUp(powerShotSpeed));
        }

        @Override
        public void update() {
            super.update();

            if(isDrivetrainManualInputActive()) { // Return manual control
                stopAutoDriving();
                stateMachine.changeState(DRIVE, new Drive_Manual_AllStates());
            }
            if(opMode.mecanumDrive.isIdle() && !arrived) {
                arrived = true;
                nextState(LAUNCHER, new Launch_fire(powerShotSpeed));
            }
            if(arrived && stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                nextState(DRIVE, nextDriveState);
            }
        }
    }

    // Restores all states to driver controlled
    static class Drive_Manual_AllStates extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            nextState(LAUNCHER, new LaunchArm_Manual());
            nextState(DRIVE, new Drive_Manual());
        }
    }

    /*
     *    launchSpeed (percent) is an argument of the
     *    constructor, so this mode works with powershots and high goal shots.
     *    However, the feedback that shows we're ready to shoot is to support moving powershots.
     */
    static class Launch_windUp extends Executive.StateBase<Manual> {
        double launchSpeed;
        double launchVelocity_tps; // encoder ticks per second

        Launch_windUp(double launchSpeed) {
            this.launchSpeed = launchSpeed;
            this.launchVelocity_tps = (0.95 * Configuration.getLaunchTicksPerSecondFromPowerSpeed(launchSpeed));
        }

        @Override
        public void update() {
            super.update();
            // Set launch speed and servo position.
            // If velocity is high enough, and servoDelay elapsed, then isDone = true
            opMode.motorUtility.setPower(Motors.LAUNCHER, this.launchSpeed);
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
            isDone = opMode.motorUtility.getVelocity(Motors.LAUNCHER) > this.launchVelocity_tps && stateTimer.seconds() > servoDelay;
        }
    }

    /*
     * Launch_fire
     */
    class Launch_fire extends Executive.StateBase<Manual> {
        double launchSpeed;
        double launchVelocity_tps; // encoder ticks per second
        boolean servoPushed = false;
        double manualServoDelay = 0.4;
        ElapsedTime servoTimer = new ElapsedTime();

        Launch_fire(double launchSpeed) {
            this.launchSpeed = launchSpeed;
            this.launchVelocity_tps = (0.95 * Configuration.getLaunchTicksPerSecondFromPowerSpeed(launchSpeed));
        }

        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.LAUNCHER, this.launchSpeed);
            if (!(servoPushed)) {
                if (opMode.motorUtility.getVelocity(Motors.LAUNCHER) > this.launchVelocity_tps) {
                    opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_PUSH_POS);
                    servoPushed = true;
                    servoTimer.reset();
                }
            } else {
                if (servoTimer.seconds() > manualServoDelay) {
                    //isDone = true; // This indicates we've shot, but not that we're ready to shoot.
                    nextState(LAUNCHER, new Launch_windUp(this.launchSpeed));
                }
            }
        }
    }
}
