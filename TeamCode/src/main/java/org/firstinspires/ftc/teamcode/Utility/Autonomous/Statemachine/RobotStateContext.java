package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Vision.DetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.LAUNCHER;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.WOBBLE;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_HIGH_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_LOW_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_MIDDLE_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.ARM_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private ShippingHubLevel level = ShippingHubLevel.TOP;
    private TrajectoryRR trajectoryRR;

    private DetectionAmount canSee = DetectionAmount.NONE;

    enum ShippingHubLevel {
        TOP,
        MIDDLE,
        BOTTOM
    }

    public RobotStateContext(AutoOpmode opmode, AllianceColor allianceColor, StartPosition startPosition) {
        this.opmode = opmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opmode);
        stateMachine.update();
    }

    public void init() {
        trajectoryRR = new TrajectoryRR(opmode.mecanumDrive);
        trajectoryRR.resetTrajectories(allianceColor);
        stateMachine.changeState(DRIVE, new Start());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
        Pose2d poseEstimate = opmode.mecanumDrive.getPoseEstimate();
        opmode.telemetry.addData("Vision:   ", canSee.name());
        opmode.telemetry.addData("X:        ", df.format(poseEstimate.getX()));
        opmode.telemetry.addData("Y:        ", df.format(poseEstimate.getY()));
        opmode.telemetry.addData("Heading:  ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(opmode.packet != null) {
            opmode.packet.put("Vision:      ", canSee.name());
        }
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStateByType();
    }

    /**
     * Start State
     * State that sets the robot's position to the start position.
     * Changes the routine based on start position.
     *
     * Trajectory: none
     * Next State: Initial
     */
    class Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            if(allianceColor.equals(AllianceColor.RED))
                trajectoryRR.resetTrajectories(AllianceColor.RED);
            switch (startPosition) {
                case CAROUSEL:
                    setupInitialPosition(trajectoryRR.getStartCarousel());
                    nextState(DRIVE, new Initial());
                    break;
                case WAREHOUSE:
                    //ToDo Use the correct position
//                    setupInitialPosition(trajectoryRR.getStartWarehouse());
                    nextState(DRIVE, new Initial());
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }
        }

        private void setupInitialPosition(Pose2d initialPosition) {
            opMode.mecanumDrive.setPoseEstimate(initialPosition);
        }
    }

    /**
     * Initial State
     * State that is completely unnecessary.
     *
     * Trajectory: none
     * Next State: Scan
     */
    class Initial extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextState(LAUNCHER, new ArmDrive());
        }

        @Override
        public void update() {
            super.update();
            if(!isDone) {
                isDone = true;
                stateTimer.reset();
            }
            if(stateTimer.seconds() > 8.0) {
                switch (startPosition) {
                    case CAROUSEL:
                        if (stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                            nextState(DRIVE, new Scan());
                        break;
                    case WAREHOUSE:
                        if (stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                            nextState(DRIVE, new Park());
                }
            }
        }
    }

    /**
     * Scan State
     *
     *
     * Trajectory: none
     * Next State:
     */
    class Scan extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            switch (opMode.initializationDetectionAmount) {
                case LEFT:
                    level = allianceColor.equals(AllianceColor.BLUE) ? ShippingHubLevel.BOTTOM : ShippingHubLevel.BOTTOM;
                    break;
                case RIGHT:
                    level = ShippingHubLevel.MIDDLE;
                    break;
                case NONE:
                    // Redundant
                    level = ShippingHubLevel.TOP;
            }

            switch (startPosition) {
                case CAROUSEL:
                    nextState(DRIVE, new StartToHub());
                    break;
                case WAREHOUSE:
            }
        }
    }

    class StartToHub extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.removeStateByType(LAUNCHER);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToHub());
        }

        @Override
        public void update() {
            super.update();
            if(!opMode.mecanumDrive.isIdle()) {
                opMode.servoUtility.setAngle(Servos.CARGO_GATE, Configuration.ServoPosition.CRADLE.getPos());
                if (stateTimer.seconds() > 0.3)
                    opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
                return;
            }

            if(!stateMachine.getCurrentStateByType(LAUNCHER).getSimpleName().equals(LiftHubLevel.class.getSimpleName()))
                nextState(LAUNCHER, new LiftHubLevel(level));

            if(stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                opMode.servoUtility.setAngle(Servos.CARGO_GATE, Configuration.ServoPosition.DROP.getPos());
                if(!isDone) {
                    stateTimer.reset();
                    isDone = true;
                }
                if(stateTimer.seconds() > 2.0) {
                    nextState(LAUNCHER, new ResetLiftToIntake());
//                    if(allianceColor.equals(AllianceColor.RED))
//                        nextState(DRIVE, new CarouselToDepotPark());
//                    else
                        nextState(DRIVE, new HubToCarousel());
                }

            }
        }
    }

    class HubToCarousel extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHubToCarousel());
        }

        @Override
        public void update() {
            super.update();
            if(!opMode.mecanumDrive.isIdle()) {
                stateTimer.reset();
                return;
            }

            opMode.motorUtility.setPower(Motors.CAROUSEL, (allianceColor.equals(AllianceColor.BLUE) ? -1 : 1) * 0.6 * Range.clip(stateTimer.seconds(), 0.0, 1.0));

            if(stateTimer.seconds() > 5.0) {
                nextState(DRIVE, new CarouselToDepotPark());
            }
        }
    }

    class CarouselToDepotPark extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
//            if(allianceColor.equals(AllianceColor.RED))
//                opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHubToDepotPark());
//            else
                opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCarouselToDepot());
        }

        @Override
        public void update() {
            super.update();

            if(opMode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new Stop());
            }
        }
    }

    static class ResetLiftToIntake extends Executive.StateBase<AutoOpmode> {
        double craneTime = 1.8;
        boolean aBoolean = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.servoUtility.setAngle(Servos.CARGO_GATE, Configuration.ServoPosition.INTAKE.getPos());
        }

        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() < craneTime)
                opMode.servoUtility.setPower(ContinuousServo.CRANE, 1.0);
            else {
                opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
                if(!isDone) {
                    if(!aBoolean) {
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 1.0);
                        aBoolean = true;
                    } else {
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
                        isDone = true;
                    }
                }
            }
            opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
        }
    }

    static class LiftHubLevel extends Executive.StateBase<AutoOpmode> {
        ShippingHubLevel hubLevel;
        double craneTime = 1.8;
        boolean armDone = false;

        LiftHubLevel(ShippingHubLevel hubLevel) {
            this.hubLevel = hubLevel;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            if(hubLevel.equals(ShippingHubLevel.MIDDLE)) craneTime = 1.5;
        }

        @Override
        public void update() {
            super.update();
            switch (hubLevel) {
                case TOP:
                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_HIGH_POS, 1.0);
                    if (stateTimer.seconds() < craneTime)
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, -1.0);
                    else
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
                    break;
                case BOTTOM:
                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
                    if (stateTimer.seconds() < craneTime)
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, -1.0);
                    else
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
                    break;
                case MIDDLE:
                    armDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_MIDDLE_POS, 1.0);
                    if (stateTimer.seconds() < craneTime)
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, -1.0);
                    else
                        opMode.servoUtility.setPower(ContinuousServo.CRANE, 0);
            }

            isDone = stateTimer.seconds() > craneTime && armDone;
        }
    }

    /**
     * State
     *
     *
     * Trajectory: none
     * Next State: Stop
     */
    class StartToCarousel extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToCarousel());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle()){
                if(!isDone) {
                    stateTimer.reset();
                    isDone = true;
                    stateMachine.changeState(WOBBLE, new CarouselRun(allianceColor.equals(AllianceColor.RED) ? 0.3:-0.3));
                }
                if(stateTimer.seconds()>4.0) {
                    nextState(WOBBLE, new Stop());
                    stateMachine.changeState(DRIVE, new CarouselToHub());
                }

            }
        }
    }

    /**
     * State
     *
     *
     * Trajectory: none
     * Next State: Stop
     */
    class CarouselToHub extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCarouselToHub());
        }

        @Override
        public void update() {
            super.update();

            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, new ResetForDriving());
        }
    }


    /**
     * Park State
     *
     *
     * Trajectory: none
     * Next State: Stop
     */
    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
//            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToPark());
        }

        @Override
        public void update() {
            super.update();

            if(opMode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new ResetForDriving());
            }
        }
    }

    /**
     * Park State
     *
     *
     * Trajectory: none
     * Next State: Stop
     */
    class ResetForDriving extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextState(LAUNCHER, new ArmPickup());
            stateMachine.removeStateByType(WOBBLE);
        }

        @Override
        public void update() {
            super.update();

            if(stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                nextState(DRIVE, new Stop());
        }
    }

    static class Stop extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            for (Executive.StateMachine.StateType type : Executive.StateMachine.StateType.values())
                stateMachine.removeStateByType(type);
            opMode.stop();
        }
    }


    static class ArmPickup extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
        }

        @Override
        public void update() {
            super.update();
            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_PICKUP_POS, 1.0);
        }
    }


    static class ArmDrive extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
        }

        @Override
        public void update() {
            super.update();
            isDone = opMode.motorUtility.goToPosition(Motors.SLIDE_ARM, ARM_LOW_POS, 1.0);
        }
    }

    static class CarouselRun extends Executive.StateBase<AutoOpmode> {
        double power;
        CarouselRun(double power) {
            this.power = power;
        }

        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.CAROUSEL, power);
        }
    }

    static class StopMotors extends  Executive.StateBase<AutoOpmode> {
        private final Motors[] motors;

        StopMotors(Motors... motors) {
            this.motors = motors;
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            for (Motors motor : motors)
                opMode.motorUtility.setPower(motor, 0);
        }
    }
}