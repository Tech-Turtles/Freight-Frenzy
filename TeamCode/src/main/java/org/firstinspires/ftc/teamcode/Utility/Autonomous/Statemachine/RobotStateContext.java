package org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.ContinuousServo;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.HardwareTypes.Servos;
import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.firstinspires.ftc.teamcode.Utility.Math.LauncherControl;
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;

import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.LAUNCHER;
import static org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive.StateMachine.StateType.WOBBLE;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.HOPPER_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.HOPPER_PUSH_POS;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.SPATULA_DOWN;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.SPATULA_STORE;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.WOBBLE_DOWN;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.WOBBLE_GRABBED_IN;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.WOBBLE_OUT_IN;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.WOBBLE_STORE;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.WOBBLE_UP;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.intakePower;
import static org.firstinspires.ftc.teamcode.Utility.Configuration.powerShotSpeed;
import static org.firstinspires.ftc.teamcode.Utility.RobotHardware.df;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final AutoOpmode opmode;
    private final Executive.StateMachine<AutoOpmode> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private TrajectoryRR trajectoryRR;

    public static boolean autoReturnToStart = false;
    public static double returnToStartDelay = 1;

    public static double servoDelay = 0.3;
    public static double wobbleOuttakeDelay = 0.75;
    public static double wobbleIntakeDelay = 0.5;
    public static double intakeDelay = 2.0;

    public static int wobbleDownOffset = -500;

    public static double wobbleArmSpeed = 1.0;
    public static double wobbleIntakeSpeed = 1.0;
    public static double highGoalSpeed_Auto = 0.55;
    public static boolean pickupRings = true;
    public static boolean doPowershot = true;
    public static boolean doSecondWobble = true;

    private RingDetectionAmount rings = RingDetectionAmount.ZERO;

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
        opmode.telemetry.addData("Rings:    ", rings.name());
        opmode.telemetry.addData("X:        ", df.format(poseEstimate.getX()));
        opmode.telemetry.addData("Y:        ", df.format(poseEstimate.getY()));
        opmode.telemetry.addData("Heading:  ", df.format(Math.toDegrees(poseEstimate.getHeading())));
        if(opmode.packet != null) {
            opmode.packet.put("Rings:       ", rings.name());
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
                case WALL:
                    setupInitialPosition(trajectoryRR.getSTART_WALL());
                    nextState(DRIVE, new Initial());
                    break;
                case CENTER:
                    setupInitialPosition(trajectoryRR.getSTART_CENTER());
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
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
            opMode.servoUtility.setAngle(Servos.SPATULA, SPATULA_STORE);
        }

        @Override
        public void update() {
            super.update();
            nextState(DRIVE, new Scan());
        }
    }

    /**
     * Scan State
     * State for setting everything associated with the amount of rings detected
     *
     * Trajectory: none
     * Next State: WallStartToCenter / (CenterStartToLeftPowershot / CenterStartToHighGoal)
     */
    class Scan extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(opMode.ringDetector != null)
                rings = opMode.ringDetector.getHeight();

            trajectoryRR.setZone(rings);

            switch (startPosition) {
                case WALL:
                    stateMachine.changeState(DRIVE, new WallStartToCenterStart());
                    break;
                case CENTER:
                    if(doPowershot)
                        stateMachine.changeState(DRIVE, new CenterStartToLeftPowershot());
                    else
                        stateMachine.changeState(DRIVE, new CenterStartToHighGoal());
            }
        }
    }

    /**
     * WallStartToCenter State
     * State that drives from the WALL_START start position, to the CENTER_START start position.
     *
     * Trajectory: TrajectoryStartWallToStartCenter
     * Next State: CenterStartToLeftPowershot
     */
    class WallStartToCenterStart extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryStartWallToStartCenter());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle()) {
                nextState(DRIVE, new CenterStartToLeftPowershot());
            }
        }
    }

    /**
     * CenterStartToLeftPowershot State
     * State that drives from the CENTER_START start position, to the left powershot.
     * The launcher winds up while driving and fires when the robot has arrived.
     *
     * Trajectory: TrajectoryParkCenterToPowershotLeft
     * Next State: LeftPowershotToCenterPowershot
     */
    class CenterStartToLeftPowershot extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCenterStartToPowershotLeft());
            nextState(LAUNCHER, new Launch_windUp(powerShotSpeed));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle() && !isDone) {
                isDone = true;
                nextState(LAUNCHER, new Launch_fire(powerShotSpeed));
            }

            if(isDone && stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                nextState(DRIVE, new LeftPowershotToCenterPowershot());
        }
    }

    /**
     * LeftPowershotToCenterPowershot State
     * State that drives from the left powershot to the center powershot.
     * The launcher winds up while driving and fires when the robot has arrived.
     *
     * Trajectory: TrajectoryPowershotLeftToPowershotCenter
     * Next State: CenterPowershotToRightPowershot
     */
    class LeftPowershotToCenterPowershot extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryPowershotLeftToPowershotCenter());
        }

       @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle() && !isDone) {
                isDone = true;
                nextState(LAUNCHER, new Launch_fire(powerShotSpeed));
            }
            if(isDone && stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                nextState(DRIVE, new CenterPowershotToRightPowershot());
            }
        }
    }

    /**
     * CenterPowershotToRightPowershot State
     * State that drives from the center powershot to the right powershot.
     * The launcher winds up while driving and fires when the robot has arrived.
     *
     * Trajectory: TrajectoryPowershotCenterPowershotRight
     * Next State: RightPowershotToWobbleDropZone
     */
    class CenterPowershotToRightPowershot extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryPowershotCenterPowershotRight());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle() && !isDone) {
                isDone = true;
                nextState(LAUNCHER, new Launch_fire(powerShotSpeed));
            }
            if(isDone && stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                nextState(LAUNCHER, new StopMotors(Motors.LAUNCHER));
                nextState(DRIVE, pickupRings && !rings.equals(RingDetectionAmount.ZERO) ? new RightPowershotToRingAlignment() : new RightPowershotToWobbleDropZone());
            }
        }
    }

    /**
     * RightPowershotToRingAlignment State
     * State that drives from the high goal position to the ring alignment position.
     *
     *
     * Trajectory: TrajectoryPowershotRightToRingPickupAlign
     * Next State: RingAlignmentToRingPickup
     */
    class RightPowershotToRingAlignment extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode>stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryPowershotRightToRingPickupAlign());
            if (rings.equals(RingDetectionAmount.FOUR))
                opMode.servoUtility.setAngle(Servos.SPATULA, SPATULA_DOWN);
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, new RingAlignmentToRingPickup());
        }
    }

    /**
     * CenterStartToHighGoal State
     * State that drives from the CENTER_START start position to the high goal position.
     *
     *
     * Trajectory: TrajectoryCenterStartToHighGoa
     * Next State: HighGoalToWobbleDropZone / HighGoalToRingAlignment
     */
    class CenterStartToHighGoal extends Executive.StateBase<AutoOpmode> {
        private int index = 0;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryCenterStartToHighGoal());
            nextState(LAUNCHER, new Launch_windUp(highGoalSpeed_Auto));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle() && !isDone) {
                isDone = true;
                nextState(LAUNCHER, new Launch_fire(highGoalSpeed_Auto));
                index++;
            }

            if(stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                isDone = false;

            if(index == 3 && opMode.mecanumDrive.isIdle() && stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                nextState(LAUNCHER, new StopMotors(Motors.LAUNCHER));
                switch (rings) {
                    case ZERO:
                        nextState(DRIVE, new HighGoalToWobbleDropZone());
                        break;
                    case ONE:
                    case FOUR:
                        if(pickupRings)
                            nextState(DRIVE, new HighGoalToRingAlignment());
                        else
                            nextState(DRIVE, new HighGoalToWobbleDropZone());
                }
            }
        }
    }

    /**
     * HighGoalToRingAlignment State
     * State that drives from the high goal position to the ring alignment position.
     *
     *
     * Trajectory: TrajectoryHighGoalToRingAlign
     * Next State: RingAlignmentToRingPickup
     */
    class HighGoalToRingAlignment extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode>stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHighGoalToRingAlign());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, new RingAlignmentToRingPickup());
        }
    }

    /**
     * RingAlignmentToRingPickup State
     * State that drives from the ring alignment position to the ring pickup position.
     *
     *
     * Trajectory: TrajectoryRingAlignToRingGrab
     * Next State: RingPickupToHighGoal
     */
    class RingAlignmentToRingPickup extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryRingAlignToRingGrab());

        }

        @Override
        public void update() {
            super.update();
            opMode.motorUtility.setPower(Motors.INTAKE, intakePower);
            if(opMode.mecanumDrive.isIdle() && timer.seconds() > intakeDelay)
                nextState(DRIVE, new RingPickupToHighGoal());
        }
    }

    /**
     * RingPickupToHighGoal State
     * State that drives from the ring pickup position to the high goal position and shoots 1/3 rings.
     *
     *
     * Trajectory: TrajectoryRingGrabToShootHighGoal
     * Next State: HighGoalToWobbleDropZone
     */
    class RingPickupToHighGoal extends Executive.StateBase<AutoOpmode> {
        private int index = 0;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryRingGrabToShootHighGoal());
            nextState(LAUNCHER, new Launch_windUp(highGoalSpeed_Auto));
            index = rings.equals(RingDetectionAmount.ONE) ? 2 : index;
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle() && !isDone) {
                isDone = true;
                nextState(LAUNCHER, new Launch_fire(highGoalSpeed_Auto));
                index++;
            }

            if(stateMachine.getStateReferenceByType(LAUNCHER).isDone)
                isDone = false;

            if(index == 3 && opMode.mecanumDrive.isIdle() && stateMachine.getStateReferenceByType(LAUNCHER).isDone) {
                opMode.servoUtility.setAngle(Servos.SPATULA, SPATULA_STORE);
                opMode.motorUtility.setPower(Motors.INTAKE, 0);
                nextState(LAUNCHER, new StopMotors(Motors.LAUNCHER));
                nextState(DRIVE, new HighGoalToWobbleDropZone());
            }
        }
    }

    /**
     * HighGoalToWobbleDropZone State
     * State that drives from the high goal position to the wobble drop zone position.
     *
     *
     * Trajectory: TrajectoryHighGoalToWobbleDropoffDeep
     * Next State: WobbleDropZoneToWobblePickupAlign
     */
    class HighGoalToWobbleDropZone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryHighGoalToWobbleDropoffDeep());
            nextState(WOBBLE, new WobblePosition(WOBBLE_DOWN + wobbleDownOffset));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;
            if(!isDone && stateMachine.getStateReferenceByType(WOBBLE).isDone) {
                nextState(INTAKE, new WobbleIntake(-wobbleIntakeSpeed));
                isDone = true;
                timer.reset();
            }

            if(isDone && timer.seconds() > wobbleOuttakeDelay || opMode.getDistance(opMode.getColorSensor(ColorSensor.WOBBLE_SENSOR)) >= WOBBLE_OUT_IN) {
                nextState(WOBBLE, new WobblePosition(WOBBLE_UP));
                nextState(DRIVE, new WobbleDropZoneToWobblePickupAlign());
            }
        }
    }

    /**
     * RightPowershotToWobbleDropZone State
     * State that drives from the right powershot to the wobble zone set in the Scan state.
     * Drops the wobble goal once it has arrived to the wobble zone.
     *
     * Trajectory: TrajectoryPowershotRightToWobbleDropOff
     * Next State: WobbleDropZoneToWobblePickupAlign
     */
    class RightPowershotToWobbleDropZone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryPowershotRightToWobbleDropOff());
            nextState(WOBBLE, new WobblePosition(WOBBLE_DOWN + wobbleDownOffset));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;
            if(!isDone && stateMachine.getStateReferenceByType(WOBBLE).isDone) {
                nextState(INTAKE, new WobbleIntake(-wobbleIntakeSpeed));
                isDone = true;
                timer.reset();
            }

            if(isDone && timer.seconds() > wobbleOuttakeDelay || opMode.getDistance(opMode.getColorSensor(ColorSensor.WOBBLE_SENSOR)) >= WOBBLE_OUT_IN) {
                nextState(WOBBLE, new WobblePosition(WOBBLE_UP));
                nextState(INTAKE, new WobbleIntake(0));
                if(doSecondWobble)
                    nextState(DRIVE, new WobbleDropZoneToWobblePickupAlign());
                else
                    nextState(DRIVE, new WobbleDropZoneToPark());
            }
        }
    }

    /**
     * WobbleDropZoneToWobblePickupAlign State
     * State that drives from the wobble zone set in the Scan state to the alignment location for
     * the second wobble goal. Lowers the wobble arm.
     *
     * Trajectory: TrajectoryWobbleDropoffToWobblePickupAlign
     * Next State: WobblePickupAlignToWobblePickup
     */
    class WobbleDropZoneToWobblePickupAlign extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWobbleDropoffToWobblePickupAlign());
        }

        @Override
        public void update() {
            super.update();
            if(timer.seconds() > 1.0 && !isDone) {
                nextState(WOBBLE, new WobblePosition(WOBBLE_DOWN));
                isDone = true;
            }

            if(opMode.mecanumDrive.isIdle() && stateMachine.getStateReferenceByType(WOBBLE).isDone) {
                nextState(INTAKE, new WobbleIntake(wobbleIntakeSpeed));
                nextState(DRIVE, new WobblePickupAlignToWobblePickup());
            }
        }
    }

    /**
     * WobblePickupAlignToWobblePickup State
     * State that drives from the alignment location for the second wobble goal to the pickup location.
     * Grabs the wobble goal and raises the wobble arm.
     *
     * Trajectory: TrajectoryWobbleAlignToWobblePickup
     * Next State: WobblePickupToWobbleDropZoneAlign
     */
    class WobblePickupAlignToWobblePickup extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWobbleAlignToWobblePickup());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;

            if(!isDone) {
                timer.reset();
                isDone = true;
            }

            if(timer.seconds() > wobbleIntakeDelay || opMode.getDistance(opMode.getColorSensor(ColorSensor.WOBBLE_SENSOR)) <= WOBBLE_GRABBED_IN) {
                nextState(INTAKE, new WobbleIntake(0));
                nextState(WOBBLE, new WobblePosition(WOBBLE_DOWN + wobbleDownOffset));
                if(opMode.getDistance(opMode.getColorSensor(ColorSensor.WOBBLE_SENSOR)) > WOBBLE_GRABBED_IN)
                    nextState(DRIVE, new WobblePickupToPark());
                else
                    nextState(DRIVE, new WobblePickupToWobbleDropZoneAlign());
            }
        }
    }

    /**
     * WobblePickupToWobbleDropZoneAlign State
     * State that drives from the second wobble goal pickup location to the wobble zone alignment.
     *
     * Trajectory: TrajectoryWobblePickupToDropoffAlign
     * Next State: WobbleDropZoneAlignToWobbleDropZone
     */
    class WobblePickupToWobbleDropZoneAlign extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWobblePickupToDropoffAlign());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, new WobbleDropZoneAlignToWobbleDropZone());
        }
    }

    /**
     * WobbleDropZoneAlignToWobbleDropZone State
     * State that drives from the wobble zone alignment to the wobble zone.
     * Drops the wobble goal.
     *
     * Trajectory: TrajectoryWobbleAlignToSecondDropoff
     * Next State: WobbleDropZoneToPark
     */
    class WobbleDropZoneAlignToWobbleDropZone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWobbleAlignToSecondDropoff());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isBusy()) return;
            if(!isDone) {
                nextState(INTAKE, new WobbleIntake(-wobbleIntakeSpeed));
                isDone = true;
                timer.reset();
            }

            if(timer.seconds() > wobbleOuttakeDelay) {
                nextState(INTAKE, new WobbleIntake(0));
                nextState(DRIVE, new WobbleDropZoneToPark());
            }
        }
    }

    /**
     * WobblePickupToPark State
     * State that drives from the second wobble pickup to the park position.
     * Used for when the robot fails to pickup the wobble goal.
     *
     * Trajectory: TrajectoryWobblePickupToPark
     * Next State: ReturnToStart / Stop
     */
    class WobblePickupToPark extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextState(WOBBLE, new WobblePosition(WOBBLE_STORE));
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectoryWobblePickupToPark());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, autoReturnToStart ? new ReturnToStart(startPosition.equals(StartPosition.CENTER) ?
                        trajectoryRR.getSTART_CENTER() : trajectoryRR.getSTART_WALL(), returnToStartDelay) : new Stop());
        }
    }

    /**
     * WobbleDropZoneToPark State
     * State that drives from the wobble zone to the park position.
     *
     * Trajectory: TrajectorySecondWobbleDropoffToPark
     * Next State: ReturnToStart / Stop
     */
    class WobbleDropZoneToPark extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            nextState(WOBBLE, new WobblePosition(WOBBLE_STORE));
            opMode.mecanumDrive.followTrajectoryAsync(trajectoryRR.getTrajectorySecondWobbleDropoffToPark());
        }

        @Override
        public void update() {
            super.update();
            if(opMode.mecanumDrive.isIdle())
                nextState(DRIVE, autoReturnToStart ? new ReturnToStart(startPosition.equals(StartPosition.CENTER) ?
                        trajectoryRR.getSTART_CENTER() : trajectoryRR.getSTART_WALL(), returnToStartDelay) : new Stop());
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

    static class ReturnToStart extends Executive.StateBase<AutoOpmode> {
        private final double delay;
        private Trajectory homeTrajectory;
        private final Pose2d endPosition;

        ReturnToStart(Pose2d endPosition, double delay) {
            this.endPosition = endPosition;
            this.delay = delay;
        }

        ReturnToStart(Pose2d endPosition) {
            this(endPosition, 0);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            homeTrajectory = opMode.mecanumDrive.trajectoryBuilder(opMode.mecanumDrive.getPoseEstimate(), Math.toRadians(180.0))
                    .lineToLinearHeading(endPosition)
                    .build();
        }

        @Override
        public void update() {
            super.update();

            if(timer.seconds() > delay && !isDone) {
                if(delay >= 0 || opMode.primary.AOnce()) {
                    isDone = true;
                    opMode.mecanumDrive.followTrajectoryAsync(homeTrajectory);
                }
            }

            if(opMode.mecanumDrive.isIdle() && isDone) {
                nextState(DRIVE, new Stop());
            }
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

    /**
    *   launchSpeed (percent) and launchVelocity_tps (ticks per second) are arguments of the
    *   constructor, so this mode works with powershots and high goal shots.
    *   However, the feedback that shows we're ready to shoot is to support moving powershots.
    */
    static class Launch_windUp extends Executive.StateBase<AutoOpmode> {
        double launchSpeed;
        double launchVelocity_tps;

        Launch_windUp(double launchSpeed) {
            this.launchSpeed = launchSpeed;
            this.launchVelocity_tps = (0.95 * Configuration.getLaunchTicksPerSecondFromPowerSpeed(launchSpeed));
        }

        @Override
        public void update() {
            super.update();
            // Set launch speed and servo position.
            // If velocity is high enough, or servoDelay elapsed, then isDone = true
            opMode.motorUtility.setPower(Motors.LAUNCHER, this.launchSpeed);
            opMode.servoUtility.setAngle(Servos.HOPPER, HOPPER_OPEN_POS);
            isDone = LauncherControl.isErrorLow(opMode.launchVelocityHistory, launchVelocity_tps) || timer.seconds() > servoDelay;
        }
    }

    static class Launch_fire extends Executive.StateBase<AutoOpmode> {
        double launchSpeed;
        double launchVelocity_tps; // encoder ticks per second
        boolean servoPushed = false;

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
                    timer.reset();
                }
            } else {
                if (timer.seconds() > servoDelay) {
                    //isDone = true; // This indicates we've shot, but not that we're ready to shoot.
                    nextState(LAUNCHER, new Launch_windUp(this.launchSpeed));
                }
            }
        }
    }

    static class WobblePosition extends Executive.StateBase<AutoOpmode> {
        private final int encoderTicks;

        WobblePosition(int encoderTicks) {
            this.encoderTicks = encoderTicks;
        }

        @Override
        public void update() {
            super.update();
            isDone = opMode.motorUtility.goToPosition(Motors.WOBBLE_ARM, encoderTicks, wobbleArmSpeed);
            if(isDone)
                opMode.motorUtility.setPower(Motors.WOBBLE_ARM, 0);
        }
    }

    static class WobbleIntake extends Executive.StateBase<AutoOpmode> {
        private final double power;

        WobbleIntake(double power) {
            this.power = power;
        }

        @Override
        public void update() {
            super.update();
            opMode.servoUtility.setPower(ContinuousServo.WOBBLE_LEFT, power);
            opMode.servoUtility.setPower(ContinuousServo.WOBBLE_RIGHT, power);
        }
    }
}