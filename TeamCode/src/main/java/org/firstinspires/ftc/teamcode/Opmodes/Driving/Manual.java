package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_DRIVE_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.cargoClose;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.cargoOpen;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.deadzone;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeExtend;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeRetract;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double driveSpeed = 1.0;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 1.0;
    public static double intakeSpeed = 1.0;
    public static double carouselSpeed = 0.55;

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

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.changeState(DRIVE, new Drive_Manual());
        stateMachine.init();
        headingController.setInputBounds(-Math.PI, Math.PI);
        mecanumDrive = new SampleMecanumDrive(hardwareMap, this);
//        trajectoryRR = new TrajectoryRR(this.mecanumDrive);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        mecanumDrive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
        mecanumDrive.update(packet);
        displayTelemetry();
    }

    /*
    Manual Control States
     */
    static class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            opMode.drivetrainStandardControls();
        }
    }

    /*
    End of Manual Control States
     */

    void drivetrainStandardControls() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Pose2d driveDirection = new Pose2d();

        Vector2d fieldFrameInput = new Vector2d(
                -gamepad1.left_stick_y * linearSpeed * precisionMode,
                -gamepad1.left_stick_x * lateralSpeed * precisionMode);

        Vector2d robotFrameInput = fieldFrameInput
                .rotated(-poseEstimate.getHeading() + Math.toRadians(90.0));

        if(primary.AOnce()) {
            precisionMode = precisionMode == 1 ? precisionPercentage : 1;
        }

//        if(primary.startOnce()) {
//            currentDriveMode = currentDriveMode == DriveMode.NORMAL_ROBOT_CENTRIC ? DriveMode.NORMAL_FIELD_CENTRIC : DriveMode.NORMAL_ROBOT_CENTRIC;
//        }

        if(primary.BOnce()) {
            servoUtility.setAngle(Servos.CARGO_GATE, Range.clip(cargoClose, 0, 1));
        } else if(primary.YOnce()) {
            servoUtility.setAngle(Servos.CARGO_GATE, Range.clip(cargoOpen, 0, 1));
        }

        if(primary.dpadUpOnce()) {
            servoUtility.setAngle(Servos.INTAKE, Range.clip(intakeExtend, 0, 1));
        } else if(primary.dpadDownOnce()) {
            servoUtility.setAngle(Servos.INTAKE, Range.clip(intakeRetract, 0, 1));
        }

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
        }

        if(currentDriveMode != DriveMode.NORMAL_FIELD_CENTRIC && Math.abs(primary.right_stick_x) > 0.1) {
            currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
        }

        mecanumDrive.setWeightedDrivePower(driveDirection);

        if(primary.rightBumper()) {
            motorUtility.setPower(Motors.INTAKE, intakeSpeed);
        } else if(primary.leftBumper()) {
            motorUtility.setPower(Motors.INTAKE, -intakeSpeed);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0.0);
        }

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.CAROUSEL, primary.right_trigger * carouselSpeed);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.CAROUSEL,  primary.left_trigger * -carouselSpeed);
        } else {
            motorUtility.setPower(Motors.CAROUSEL, 0.0);
        }
        if(secondary.left_stick_y > 0.2)
            stateMachine.changeState(LAUNCHER, new ArmManual());
        else if(primary.dpadLeftOnce())
            stateMachine.changeState(LAUNCHER, new ArmPickup());
        else if(primary.dpadRightOnce())
            stateMachine.changeState(LAUNCHER, new ArmDrive());
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

    boolean isDrivetrainManualInputActive() {
        double threshold = 0.3;
        return (Math.abs(primary.left_stick_x) > threshold)
            || (Math.abs(primary.left_stick_y) > threshold)
            || (Math.abs(primary.right_stick_x) > threshold);
    }

    void displayTelemetry() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

        telemetry.addData("Precision mode:      ", df.format(precisionMode));
        telemetry.addData("X:                   ", df.format(poseEstimate.getX()));
        telemetry.addData("Y:                   ", df.format(poseEstimate.getY()));
        telemetry.addData("Heading:             ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(packet != null) {
            packet.put("Precision mode:      ", df.format(precisionMode));
            packet.put("Drive speed:         ", df.format(driveSpeed));
            packet.put("Precision speed:     ", df.format(precisionPercentage));
            packet.put("Loop time:           ", df_precise.format(period.getAveragePeriodSec()) + "s");
        }
    }

    void stopAutoDriving() {
        mecanumDrive.cancelFollowing();
        mecanumDrive.setDrivePower(new Pose2d());
    }

    static class ArmPickup extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(opMode.secondary.left_stick_y > 0.2)
                opMode.stateMachine.changeState(LAUNCHER, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_PICKUP_POS, 1.0);
        }
    }

    static class ArmDrive extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(opMode.secondary.left_stick_y > 0.2)
                opMode.stateMachine.changeState(LAUNCHER, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_DRIVE_POS, 1.0);
        }
    }

    static class ArmManual extends Executive.StateBase<Manual> {
        int armPos = 0;
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            armPos = opMode.motorUtility.getEncoderValue(Motors.SLIDE_ARM);
        }

        @Override
        public void update() {
            super.update();
            armPos += (int) opMode.secondary.left_stick_y * 5;
            opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, armPos, 1.0);
        }
    }

    static class Stop extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
        }
    }

}
