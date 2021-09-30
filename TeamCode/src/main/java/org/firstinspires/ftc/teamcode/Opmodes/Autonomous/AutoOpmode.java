package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.Executive;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Statemachine.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount;


public class AutoOpmode extends RobotHardware {

    AllianceColor robotColor;
    StartPosition robotStartPos;
    private Executive.RobotStateMachineContextInterface robotStateContext;
    public RingDetectionAmount initializationRingDetectionAmount = null;

    @Autonomous(name="Wall Red", group="A")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Red", group="A")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Autonomous(name="Wall Blue", group="A")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.WALL;
            super.init();
        }
    }

    @Autonomous(name="Center Blue", group="A")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.CENTER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        robotStateContext = new RobotStateContext(this, robotColor, robotStartPos);
        new Thread(this::loadVision).start();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        motorUtility.setPIDFCoefficientsCompensated(Motors.LAUNCHER, DcMotor.RunMode.RUN_USING_ENCODER, Configuration.pidfCoeffFeedForward);

        robotStateContext.init();
        telemetry.addData("Initialization: ", "Successful!");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary.update();

        if(ringDetector != null) {
            initializationRingDetectionAmount = ringDetector.getHeight();
            telemetry.addData("Initialization Ring Amount", initializationRingDetectionAmount.name());
            if(packet != null)
                packet.put("Initialization Ring Amount", initializationRingDetectionAmount.name());
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.update(packet);
        robotStateContext.update();
        if(packet != null) {
            packet.put("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
            packet.put("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");
            packet.put("State:          ", robotStateContext.getCurrentState());
        }
    }
}