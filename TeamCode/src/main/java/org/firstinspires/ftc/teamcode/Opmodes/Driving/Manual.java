package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_DRIVE_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_HIGH_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_LOW_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_MAX;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.deadzone;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeExtend;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakeRetract;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ServoPosition;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double driveSpeed = 1.0;
    public static double precisionMode = 1.0;
    public static double precisionPercentage = 0.35;
    public static double linearSpeed = 1.0;
    public static double lateralSpeed = 1.0;
    public static double rotationSpeed = 0.8;
    public static double intakeSpeed = 0.85;
    public static double carouselSpeed = 0.75;
    public static double cranePower = 0;
    public static int direction = 1;
    public static ServoPosition cargoPosition = Configuration.ServoPosition.INTAKE;
    public static double cargoClose = 0.0;
    public static double cargoOpen = 0.35;


    private final Executive.StateMachine<Manual> stateMachine;
    private TrajectoryRR trajectoryRR;

    private Pose2d saveLocation = new Pose2d();

    // Align to Point code
    enum DriveMode {
        NORMAL_ROBOT_CENTRIC,
        NORMAL_FIELD_CENTRIC
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
        servoUtility.setAngle(Servos.INTAKE, intakeRetract);
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
        servoUtility.setAngle(Servos.INTAKE, intakeRetract + 0.2);
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

        if(primary.YOnce()) {
            direction = -direction;
        }

//        if(primary.startOnce())
//            currentDriveMode = currentDriveMode == DriveMode.NORMAL_ROBOT_CENTRIC ? DriveMode.NORMAL_FIELD_CENTRIC : DriveMode.NORMAL_ROBOT_CENTRIC;

        if(primary.XOnce()) {
            servoUtility.setAngle(Servos.INTAKE, Range.clip(intakeExtend, 0, 1));
        } else if(primary.BOnce()) {
            servoUtility.setAngle(Servos.INTAKE, Range.clip(intakeRetract + 0.2, 0, 1));
        }

        switch (currentDriveMode) {
            case NORMAL_ROBOT_CENTRIC:
                driveDirection = new Pose2d(
                        (-primary.left_stick_y * direction) * linearSpeed * precisionMode,
                        (-primary.left_stick_x * direction) * lateralSpeed * precisionMode,
                        -primary.right_stick_x * rotationSpeed * precisionMode
                );
                break;
            case NORMAL_FIELD_CENTRIC:
                driveDirection = new Pose2d(
                        robotFrameInput.getX(), robotFrameInput.getY(),
                        -primary.right_stick_x * rotationSpeed * precisionMode
                );
                break;
        }

        if(currentDriveMode != DriveMode.NORMAL_FIELD_CENTRIC && Math.abs(primary.right_stick_x) > 0.1) {
            currentDriveMode = DriveMode.NORMAL_ROBOT_CENTRIC;
        }

        mecanumDrive.setWeightedDrivePower(driveDirection);

        if(primary.right_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, intakeSpeed);
        } else if(primary.left_trigger > deadzone) {
            motorUtility.setPower(Motors.INTAKE, -intakeSpeed);
        } else {
            motorUtility.setPower(Motors.INTAKE, 0.0);
        }

        if(secondary.rightBumperOnce()) {
            switch (cargoPosition) {
                case INTAKE:
                    cargoPosition = ServoPosition.CRADLE;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoClose);
                    break;
                case CRADLE:
                    cargoPosition = ServoPosition.DROP;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoOpen);
                    break;
                case DROP:
                    cargoPosition = ServoPosition.INTAKE;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoOpen);
            }
        } else if(secondary.leftBumperOnce()) {
            switch (cargoPosition) {
                case INTAKE:
                    cargoPosition = ServoPosition.DROP;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoOpen);
                    break;
                case CRADLE:
                    cargoPosition = ServoPosition.INTAKE;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoOpen);
                    break;
                case DROP:
                    cargoPosition = ServoPosition.CRADLE;
                    servoUtility.setAngle(Servos.CARGO_GATE, cargoClose);
            }
        }
        if(secondary.dpadUpOnce()) {
            servoUtility.setAngle(Servos.CARGO_GATE, cargoOpen);
        } else if(secondary.dpadDownOnce())
            servoUtility.setAngle(Servos.CARGO_GATE, cargoClose);
        servoUtility.setAngle(Servos.BASKET, cargoPosition.getPos());
        servoUtility.setPower(ContinuousServo.CRANE, secondary.right_stick_y);

        if(secondary.right_trigger > deadzone)
            motorUtility.setPower(Motors.CAROUSEL, secondary.right_trigger * carouselSpeed);
        else if(secondary.left_trigger > deadzone)
            motorUtility.setPower(Motors.CAROUSEL, secondary.left_trigger * -carouselSpeed);
        else
            motorUtility.setPower(Motors.CAROUSEL, 0);

        if(Math.abs(secondary.left_stick_y) > 0.2)
            stateMachine.changeState(SLIDE, new ArmManual());
        else if(secondary.XOnce())
            stateMachine.changeState(SLIDE, new ArmGrab());
        else if(secondary.AOnce())
            stateMachine.changeState(SLIDE, new ArmIntake());
//        else if(secondary.XOnce())
//            stateMachine.changeState(LAUNCHER, new ArmDrive());
        else if(secondary.BOnce())
            stateMachine.changeState(SLIDE, new ArmLow());
        else if(secondary.YOnce())
            stateMachine.changeState(SLIDE, new ArmHigh());
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

    static class ArmIntake extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2)
                opMode.stateMachine.changeState(SLIDE, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_PICKUP_POS, 0.8);
        }
    }

    static class ArmDrive extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2)
                opMode.stateMachine.changeState(SLIDE, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_DRIVE_POS, 0.8);
        }
    }

    static class ArmHigh extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2)
                opMode.stateMachine.changeState(SLIDE, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_HIGH_POS, 0.8);
        }
    }

    static class ArmLow extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2)
                opMode.stateMachine.changeState(SLIDE, new ArmManual());
            else
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 0.8);
        }
    }

    static class ArmManual extends Executive.StateBase<Manual> {
        int armPos;

        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            armPos = opMode.motorUtility.getEncoderValue(Motors.SLIDE_ARM);
        }

        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2) {
                if (opMode.secondary.left_stick_y < 0.2 || opMode.motorUtility.getEncoderValue(Motors.SLIDE_ARM) < ARM_MAX) {
                    opMode.motorUtility.setPower(Motors.SLIDE_ARM, -opMode.secondary.left_stick_y*0.8);
                    armPos = opMode.motorUtility.getEncoderValue(Motors.SLIDE_ARM);
                }
            } else {
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, armPos, 1.0);
            }
        }
    }

    static class ArmGrab extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
            if(Math.abs(opMode.secondary.left_stick_y) > 0.2)
                opMode.stateMachine.changeState(SLIDE, new ArmManual());
            else {
                opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS + 300, 1.0);
                cargoPosition = ServoPosition.CRADLE;
                opMode.servoUtility.setAngle(Servos.CARGO_GATE, cargoClose);
            }
        }
    }

    static class Stop extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();
        }
    }

}
