package org.firstinspires.ftc.teamcode.Opmodes.Driving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareTypes.*;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.Utility.Roadrunner.util.TelemetryLog;

import java.io.File;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

/**
 * @author Christian
 * An opmode that shows as much data about the robot as possible.
 */
@TeleOp(name="Diagnostic", group="C")
public class Diagnostic extends Manual {

    List<TelemetryLog> telemetryLogList = new ArrayList<>();

    @Override
    public void init() {
        super.init();
        imuUtil = new IMUUtilities(this, IMU.IMU1.getName(), IMUUtilities.ImuMode.FAST_HEADING_ONLY);
        telemetry.addLine("\n----Diagnostic Initialized----");
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        telemetryLogList.add(new TelemetryLog(this));

        for (MotorTypes type : MotorTypes.values()) {
            telemetry.addLine(type.name());
            for (Motors motor : Motors.values()) {
                if(type != motor.getType()) continue;
                telemetry.addData(motor.name() + ": ", motorUtility.getEncoderValue(motor));
            }
        }

        for (Servos servo : Servos.values()) {
            telemetry.addData(servo.name() + ": ", servoUtility.getAngle(servo));
        }

        telemetry.addData("Period Average: ", df_precise.format(period.getAveragePeriodSec()) + "s");
        telemetry.addData("Period Max:     ", df_precise.format(period.getMaxPeriodSec()) + "s");

        if(imuUtil.imu != null) {
            imuUtil.updateNow();
            imuUtil.displayTelemetry();
        }
    }

    @Override
    public void stop() {
        super.stop();
        File logFile = LoggingUtil.getLogFile("debugLogFile.csv");
        LoggingUtil.saveTelemetryLogListToFile(logFile, telemetryLogList);
    }
}