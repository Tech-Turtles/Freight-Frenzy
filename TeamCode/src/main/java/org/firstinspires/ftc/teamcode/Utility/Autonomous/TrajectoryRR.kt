package org.firstinspires.ftc.teamcode.Utility.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.firstinspires.ftc.teamcode.Utility.Odometry.DriveConstants
import org.firstinspires.ftc.teamcode.Utility.Odometry.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount
import org.firstinspires.ftc.teamcode.Utility.Vision.RingDetectionAmount.*
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class TrajectoryRR constructor(sampleMecanumDrive: SampleMecanumDrive){
    val drive: SampleMecanumDrive = sampleMecanumDrive

    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var slowVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var slowAccelerationConstraint: TrajectoryAccelerationConstraint? = null
    private var ringVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var ringAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private val slowVelocity: Double = 10.0
    private val slowAcceleration: Double = 40.0
    private val ringVelocity: Double = 60.0
    private val ringAcceleration: Double = 40.0

    val list = ArrayList<Trajectory>()

    fun setZone(ringAmount: RingDetectionAmount) {

        buildtrajectories()
    }


    init {
        velocityConstraint = getMinVelocityConstraint(DriveConstants.MAX_VEL)
        accelerationConstraint = getMinAccelerationConstraint(DriveConstants.MAX_ACCEL)

        slowVelocityConstraint = getMinVelocityConstraint(slowVelocity)
        slowAccelerationConstraint = getMinAccelerationConstraint(slowAcceleration)

        ringVelocityConstraint = getMinVelocityConstraint(ringVelocity)
        ringAccelerationConstraint = getMinAccelerationConstraint(ringAcceleration)

        buildtrajectories()
        setZone(ZERO)
    }

    private fun buildtrajectories() {

        // Example
//        val tempTraj: Trajectory =
//                trajectoryBuilder(STARTPOS, (90.0 - 20.0).toRadians)
//                        .splineToConstantHeading(CENTER.vec(), (20.0 + 90.0).toRadians)
//                        .build()
//        this.mainTraj = tempTraj
    }

    fun toVector2d(pose: Pose2d): Vector2d {
        return Vector2d(pose.x, pose.y)
    }

    fun  trajectoryBuilder(pose: Pose2d, heading: Double): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, heading)
    }

    fun  trajectoryBuilder(pose: Pose2d, reversed: Boolean): TrajectoryBuilder{
        return drive.trajectoryBuilder(pose, reversed)
    }

    companion object {
        @JvmStatic
        fun getNearestCornerPose2d(pose: Pose2d): Pose2d {
            val flipOffset = Pose2d(0.75, 0.0, 180.0.toRadians)
            val corners = ArrayList<Pose2d>()
            corners.add(Pose2d(-61.5, -61.25, (0.0).toRadians))
            // TODO: re-enable other 3 corners
//            corners.add(Pose2d( 60.75, -61.0, (0.0).toRadians))
//            corners.add(Pose2d(-61.5, 61.0, (0.0).toRadians))
//            corners.add(Pose2d( 60.75, 61.0, (0.0).toRadians))

            // Which direction, fwd 0.0 or reverse 180.0?
            // TODO: re-enable forward orientation
            val isHeadingFwd = false // HARDCODED
            //val isHeadingFwd = Math.abs(0.0.toRadians - pose.heading) < 90.0.toRadians

            val orientationOffset =
                if(isHeadingFwd) Pose2d(0.0, 0.0, 0.0)
                else flipOffset

            // Which point is the closest?
            var closestCorner = corners[0].plus(orientationOffset)
            var closestDistance = getDistance(pose, closestCorner)
            var newCorner: Pose2d?
            var newDistance: Double?
            for(corner in corners) {
                newCorner = corner
                newDistance = getDistance(pose, newCorner)
                if(newDistance < closestDistance) {
                    closestCorner = newCorner.plus(orientationOffset)
                    closestDistance = newDistance
                }
            }
            return closestCorner
        }

        private fun getDistance(pose1: Pose2d, pose2: Pose2d): Double {
            val deltaX = abs(pose1.x - pose2.x)
            val deltaY = abs(pose1.y - pose2.y)
            return sqrt(deltaX.pow(2.0) + deltaY.pow(2.0))
        }
    }

    private fun getMinVelocityConstraint(MaxVelocity: Double): MinVelocityConstraint {
        return MinVelocityConstraint(listOf(
                AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                MecanumVelocityConstraint(MaxVelocity, DriveConstants.TRACK_WIDTH)
        ))
    }

    private fun getMinAccelerationConstraint(MaxAccel: Double): ProfileAccelerationConstraint {
        return ProfileAccelerationConstraint(MaxAccel)
    }
}

val Double.toRadians get() = (Math.toRadians(this))

/*
    +x is the 'positive' direction, and rotation is counter-clockwise around (0,0)
    https://en.wikipedia.org/wiki/Rotation_matrix
 */
fun Pose2d.rotateFrame(rotationRadians: Double): Pose2d
{
    return Pose2d(this.x * kotlin.math.cos(rotationRadians) - this.y * kotlin.math.sin(rotationRadians),
            this.x * kotlin.math.sin(rotationRadians) + this.y * kotlin.math.cos(rotationRadians),
            this.heading + rotationRadians)
}