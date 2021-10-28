package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Vision.DetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private TrajectoryRR trajectoryRR;

    private DetectionAmount canSee = DetectionAmount.NONE;

    public RobotStateContext(AutoOpmode opmode, AllianceColor allianceColor, StartPosition startPosition) {
        this.opmode = opmode;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(opmode);
        stateMachine.update();
    }

    public void init() {
        trajectoryRR = new TrajectoryRR(opmode.mecanumDrive);
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
            switch (startPosition) {
                case CAROUSEL:
                    nextState(DRIVE, new Initial());
                    break;
                case DEPOT:
                    nextState(DRIVE, new Park());
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
        }

        @Override
        public void update() {
            super.update();
            nextState(DRIVE, new Scan());
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

            if(opMode.ringDetector != null)
                canSee = opMode.ringDetector.getHeight();

            switch (startPosition) {
                case CAROUSEL:

                    break;
                case DEPOT:

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
    class Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartToPark());
        }

        @Override
        public void update() {
            super.update();

            if(opMode.mecanumDrive.isIdle())
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