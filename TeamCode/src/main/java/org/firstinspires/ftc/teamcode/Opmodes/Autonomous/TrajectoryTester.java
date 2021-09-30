package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Autonomous.TrajectoryRR;
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.util.ArrayList;

@Autonomous(name = "Trajectory Tester", group = "E")
public class TrajectoryTester extends RobotHardware {
    TrajectoryRR trajectoryRR = null;
    Pose2d startPosition = new Pose2d();
    Trajectory routine;
    ArrayList<Trajectory> trajectories = new ArrayList<>();
    int index = 0;

    @Override
    public void init() {
        super.init();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));
        trajectoryRR = new TrajectoryRR(mecanumDrive);
        startPosition = trajectoryRR.getSTART_CENTER();
        mecanumDrive.setPoseEstimate(startPosition);
//        trajectories.add(0, trajectoryRR.getTrajCenterStartToHighGoal());
//        trajectories.add(1, trajectoryRR.getTrajHighGoalToRingAlign());
//        trajectories.add(2, trajectoryRR.getTrajRingAlignToRingGrab());
//        trajectories.add(3, trajectoryRR.getTrajRingGrabToShootHighGoal());
//        trajectories.add(4, trajectoryRR.getTrajHighGoalToWobbleDropoffDeep());
        routine = trajectories.get(index);
        index++;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        if(primary.AOnce()) {
            try {
                routine = trajectories.get(index);
            } catch (IndexOutOfBoundsException ignore) {}
            index++;
        }

        if(primary.BOnce() && !(index >= trajectories.size())) {
            mecanumDrive.followTrajectoryAsync(routine);
        }
        mecanumDrive.update(packet);
    }
}