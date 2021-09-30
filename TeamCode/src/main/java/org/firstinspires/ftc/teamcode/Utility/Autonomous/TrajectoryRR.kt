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

    /*
   Offset system
    */
    private val offsetWobbleArmReach: Double = 12.0 + 4.0
    private val wobbleDropoffLateral: Double = 3.0
    private val wobblePickupLateral: Double = -3.0
    private val offsetWobbleDropoffAlign = Pose2d(-offsetWobbleArmReach - 6.0, wobbleDropoffLateral, 0.0)
    private val offsetWobbleDropoffDeep = Pose2d(-offsetWobbleArmReach + 2.0, wobbleDropoffLateral, 0.0)
    private val offsetWobbleDropoffShallow = Pose2d(-offsetWobbleArmReach - 2.0, wobbleDropoffLateral, 0.0)
    private val wobbleDropoffRotationRadians: Double = 0.0.toRadians

    private val zoneBShallowOffset = Pose2d(-5.0, -2.0, 0.0)
    private val zoneCShallowOffset = Pose2d(0.0, 5.0, 0.0)
    private val zoneBDeepOffset =  Pose2d(3.0, 3.5 - 1.0, 0.0)
    private val zoneCDeepOffset =  Pose2d(0.0, 0.0, 0.0)

    private val offsetWobblePickupAlign = Pose2d(-offsetWobbleArmReach - 8.0 + 6.0, wobblePickupLateral, 0.0)
    private val offsetWobblePickupGrab = Pose2d(-offsetWobbleArmReach + 3.0 + 4.0 + 3.0, wobblePickupLateral, 0.0) // Drive through wobble goal
    private val wobblePickupRotationRadians: Double = (165.0).toRadians

    private val offsetRingPickupAlign = Pose2d(-18.0, -5.0, 0.0)
    private val offsetRingPickupGrab = Pose2d(10.0, -5.0, 0.0)
    private val ringPickupRotationRadians: Double = (170.0).toRadians

    private val spacingPowershot: Double = 7.0 // Spacing between the powershot sticks in the y axis, inches

    private val wobbleOffset: Pose2d = Pose2d(-12.0, 0.0, 0.0)
    private val wobblePickup: Pose2d = Pose2d(12.0, 0.0, 0.0)

    // Actual locations before offsets for robot grabber
    private var RINGS_ACTUAL = Pose2d(-24.0, -36.0, Math.toRadians(0.0))  // Needs rotation for pickup
    private var WOBBLE_PICKUP_ACTUAL = Pose2d(-48.0, -50.0, Math.toRadians(0.0)) // Needs rotation for pickup
    private var ZONE_A_CENTER = Pose2d(12.0, -60.0, Math.toRadians(0.0))
    private var ZONE_B_CENTER = Pose2d(36.0, -36.0, Math.toRadians(0.0))
    private var ZONE_C_CENTER = Pose2d(60.0, -60.0, Math.toRadians(0.0))

    // Start Positions
    // True X is -60.75, not -62.0
    //TODO Change start position and update old positions with the new value
    val START_WALL = Pose2d(-62.0, -42.0, Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -18.0, Math.toRadians(180.0 + 0.0))


    var SHOOT_HIGHGOAL      = Pose2d(-4.0 - 1.5, -42.0 + 2.0 + 10.0 - 4.0, Math.toRadians(180.0 - 0.0))
    var POWERSHOT_LEFT      = Pose2d(-4.0, -6.5 + spacingPowershot + -3.0 + 1.0, Math.toRadians(180.0 - 0.0))
    var POWERSHOT_CENTER    = POWERSHOT_LEFT.plus(Pose2d(0.0, -1.0 * spacingPowershot - 1.5 + 2.0 - 1.0 -0.75, 0.0))
    var POWERSHOT_RIGHT     = POWERSHOT_LEFT.plus(Pose2d(0.0, -2.0 * spacingPowershot - 1.5 + 3.0 - 2.5 , 0.0))


    // Variable waypoints
    var ZONE_CENTER_VARIABLE = ZONE_A_CENTER
    var wobbleDropoffAlign: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobbleDropoffDeep: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobbleDropoffShallow: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(
            wobbleDropoffRotationRadians))
    var wobblePickupAlign: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(
            wobblePickupRotationRadians))
    var wobblePickupGrab: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(
            wobblePickupRotationRadians))
    var ringPickupAlign: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
    var ringPickupGrab: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))

    var ZONE_A = ZONE_A_CENTER.plus(wobbleOffset)
    var ZONE_B = ZONE_B_CENTER.plus(wobbleOffset)
    var ZONE_C = ZONE_C_CENTER.plus(wobbleOffset)
    var PARK = Pose2d(12.0, -42.0, Math.toRadians(180.0))
    var FAR_PARK = Pose2d(12.0, -15.0, Math.toRadians(180.0))
    var WOBBLE_WALL = Pose2d(-48.0, -50.0, Math.toRadians(180.0)).plus(wobblePickup)


    var WALL_WAY = Pose2d(-24.0, -56.0, Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0, 4.0, 0.0))
    var CENTER_WAY_START = Pose2d(-36.0, 0.0, 180.0.toRadians)
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C
    var wobbleTangent: Double = -45.0


    var trajectoryStartWallToStartCenter: Trajectory? = null

    // Powershots split into separate steps
    var trajectoryCenterStartToPowershotLeft: Trajectory? = null
    var trajectoryPowershotLeftToPowershotCenter: Trajectory? = null
    var trajectoryPowershotCenterPowershotRight: Trajectory? = null
    var trajectoryPowershotRightToWobbleDropOff: Trajectory? = null

    // New trajectories
    var trajectoryWobbleDropoffToWobblePickupAlign: Trajectory? = null
    var trajectoryWobbleAlignToWobblePickup: Trajectory? = null
    var trajectoryWobblePickupToPark: Trajectory? = null
    var trajectoryWobblePickupToDropoffAlign: Trajectory? = null
    var trajectoryWobbleAlignToSecondDropoff: Trajectory? = null
    var trajectorySecondWobbleDropoffToPark: Trajectory? = null
    var trajectorySecondWobbleDropoffToRingPickupAlign: Trajectory? = null
    var trajectoryRingAlignToRingGrab: Trajectory? = null
    var trajectoryRingGrabToShootHighGoal: Trajectory? = null
    var trajectoryFromShootHighGoalToPark: Trajectory? = null


    // Pickup rings after powershot but before first wobble pickup
    // trajectoryPowershotRightToWobbleDropoff  defined above
    var trajectoryPowershotRightToRingPickupAlign: Trajectory? = null

    // 5 trajectories to support high goal and high goal + ring pickup trajectories
    var trajectoryCenterStartToHighGoal: Trajectory? = null
    var trajectoryHighGoalToRingAlign: Trajectory? = null
    // trajectoryRingAlignToRingGrab     already defined
    // trajectoryRingGrabToShootHighGoal already defined
    var trajectoryHighGoalToWobbleDropoffDeep: Trajectory? = null

    var trajectoryCenterStartToPark: Trajectory? = null



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
        ZONE_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A
            ONE -> ZONE_B
            FOUR -> ZONE_C
        }
        ZONE_CENTER_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A_CENTER
            ONE -> ZONE_B_CENTER
            FOUR -> ZONE_C_CENTER
        }
        wobbleTangent = when (ringAmount) {
            ZERO -> -40.0
            ONE -> 0.0
            FOUR -> -15.0
        }

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

        // Demo relative start position placement
        val trajectoryStartWallToStartCenter: Trajectory =
                trajectoryBuilder(START_WALL, (90.0 - 20.0).toRadians)
                        .splineToConstantHeading(START_CENTER.vec(), (20.0 + 90.0).toRadians)
                        .build()
        this.trajectoryStartWallToStartCenter = trajectoryStartWallToStartCenter

        // Recalculate class variable waypoints based on ZONE_CENTER_VARIABLE
        wobbleDropoffAlign = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffDeep = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffShallow = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(wobbleDropoffRotationRadians))
        wobblePickupAlign = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(wobblePickupRotationRadians))
        wobblePickupGrab = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(wobblePickupRotationRadians))
        ringPickupAlign = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
        ringPickupGrab = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))

        val trajectoryCenterStartToPowershotLeft: Trajectory=
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        //.splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_LEFT.vec())
                        .build()
        this.trajectoryCenterStartToPowershotLeft = trajectoryCenterStartToPowershotLeft


        val trajectoryPowershotLeftToPowershotCenter: Trajectory=
                trajectoryBuilder(trajectoryCenterStartToPowershotLeft.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_CENTER.vec())
                        .build()
        this.trajectoryPowershotLeftToPowershotCenter = trajectoryPowershotLeftToPowershotCenter


        val trajectoryPowershotCenterPowershotRight: Trajectory=
                trajectoryBuilder(trajectoryPowershotLeftToPowershotCenter.end(), -90.0.toRadians)
                        .lineToConstantHeading(POWERSHOT_RIGHT.vec())
                        .build()
        this.trajectoryPowershotCenterPowershotRight = trajectoryPowershotCenterPowershotRight


        /*
             Option: From powershot_right go to ring pickup
         */
        // Powershot Right to Ring Align
        val trajectoryPowershotRightToRingPickupAlign: Trajectory=
                trajectoryBuilder(trajectoryPowershotCenterPowershotRight.end(), -90.0.toRadians)
                        .lineToLinearHeading(ringPickupAlign)
                        .build()
        this.trajectoryPowershotRightToRingPickupAlign = trajectoryPowershotRightToRingPickupAlign


        /*
            If skipping rings, go straight to wobble dropoff
         */
        val trajectoryPowershotRightToWobbleDropoff: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryPowershotCenterPowershotRight.end(), (-90.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0, 6.0, 0.0)), Math.toRadians(-90.0))
                                .lineToConstantHeading(wobbleDropoffDeep.vec())
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajectoryPowershotCenterPowershotRight.end(), (-50.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBDeepOffset), 0.0)
                                .lineToConstantHeading(wobbleDropoffDeep.plus(zoneBDeepOffset).vec())
                                .build()
                    else -> // Zone C
                        trajectoryBuilder(trajectoryPowershotCenterPowershotRight.end(), 0.0.toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
//                                .lineToLinearHeading(wobbleDropoffDeep)
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-15.0, 5.0, 1.0.toRadians)).plus(zoneCDeepOffset), (-20.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(3.0, 0.0 , 0.0)).plus(zoneCDeepOffset), (-20.0).toRadians)
                                .build()
                }
        this.trajectoryPowershotRightToWobbleDropOff = trajectoryPowershotRightToWobbleDropoff

        // From zone wobble dropoff position to rings pickup align
        val trajectoryWobbleDropoffToWobblePickupAlign: Trajectory=
                when (ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryPowershotRightToWobbleDropoff.end(), 120.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajectoryPowershotRightToWobbleDropoff.end(), 180.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                    else ->
                        trajectoryBuilder(trajectoryPowershotRightToWobbleDropoff.end(), 150.0.toRadians)
                                .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                                .build()
                }
        this.trajectoryWobbleDropoffToWobblePickupAlign = trajectoryWobbleDropoffToWobblePickupAlign


        // Grab Wobble Goal
        val trajectoryWobbleAlignToWobblePickup: Trajectory=
                trajectoryBuilder(trajectoryWobbleDropoffToWobblePickupAlign.end(), 0.0.toRadians)
                        .lineToConstantHeading(wobblePickupGrab.vec(), slowVelocityConstraint, slowAccelerationConstraint)
                        .build()
        this.trajectoryWobbleAlignToWobblePickup = trajectoryWobbleAlignToWobblePickup

        val trajectoryWobblePickupToPark: Trajectory =
                trajectoryBuilder(trajectoryWobbleAlignToWobblePickup.end(), false)
                        .lineToLinearHeading(FAR_PARK)
                        .build()
        this.trajectoryWobblePickupToPark = trajectoryWobblePickupToPark

        // Align to dropoff second wobble goal
        val trajectoryWobblePickupToDropoffAlign: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                                .splineToSplineHeading(wobbleDropoffAlign, Math.toRadians(-30.0))
//                                .lineToConstantHeading(wobbleDropoffAlign.vec())
                                .build()
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajectoryWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                                .lineToConstantHeading(wobblePickupAlign.plus(zoneBShallowOffset).vec())
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBShallowOffset), 25.0.toRadians)
//                                .lineToConstantHeading(wobbleDropoffDeep.vec())
                                .build()
                    else -> // Zone C
                        trajectoryBuilder(trajectoryWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + (180.0).toRadians)
//                                .lineToConstantHeading(wobblePickupAlign.vec())
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneCShallowOffset), (-20.0).toRadians)
                                .build()
                }
        this.trajectoryWobblePickupToDropoffAlign = trajectoryWobblePickupToDropoffAlign

        // Dropoff Second WobbleGoal
        val trajectoryWobbleAlignToSecondDropoff: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajectoryWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffShallow.plus(zoneBShallowOffset).vec())
                                .build()
                    ZONE_C_CENTER ->
                        trajectoryBuilder(trajectoryWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffShallow.plus(zoneCShallowOffset).vec())
                                .build()
                    else ->
                        trajectoryBuilder(trajectoryWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffShallow.vec())
                                .build()
                }
        this.trajectoryWobbleAlignToSecondDropoff = trajectoryWobbleAlignToSecondDropoff


        // Second WobbleGoal to Park (safe option)
        val trajectorySecondWobbleDropoffToPark: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffAlign.vec())
                                .splineToConstantHeading(PARK.vec().plus(Vector2d(0.0, 12.0)), 0.0.toRadians)
                                .build()
                    else ->
                        trajectoryBuilder(trajectoryWobbleAlignToSecondDropoff.end(), 135.0.toRadians)
                                .lineToConstantHeading(PARK.vec().plus(Vector2d(-8.0, 12.0)))
                                .build()
                }
        this.trajectorySecondWobbleDropoffToPark = trajectorySecondWobbleDropoffToPark


        // Second WobbleGoal to Ring Pickup Align
        val trajectorySecondWobbleDropoffToRingPickupAlign: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToConstantHeading(wobbleDropoffAlign.vec().plus(Vector2d(0.0, 7.0)))
                                .splineToSplineHeading(ringPickupAlign, 90.0.toRadians + 0.0 * ringPickupRotationRadians)
//                                .lineToLinearHeading(ringPickupAlign)
                                .build()
                    else ->
                        trajectoryBuilder(trajectoryWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                                .lineToLinearHeading(ringPickupAlign)
                                .build()
                }
        this.trajectorySecondWobbleDropoffToRingPickupAlign = trajectorySecondWobbleDropoffToRingPickupAlign


        // Pickup Rings
        val trajectoryRingAlignToRingGrab: Trajectory=
                trajectoryBuilder(ringPickupAlign, 0.0.toRadians)
                        .lineToConstantHeading(ringPickupGrab.vec(), ringVelocityConstraint, ringAccelerationConstraint)
                        .build()
        this.trajectoryRingAlignToRingGrab = trajectoryRingAlignToRingGrab


        // Take picked up rings to shoot
        val trajectoryRingGrabToShootHighGoal: Trajectory=
                trajectoryBuilder(ringPickupGrab, 0.0.toRadians)
                        .lineToLinearHeading(SHOOT_HIGHGOAL)
                        .build()
        this.trajectoryRingGrabToShootHighGoal = trajectoryRingGrabToShootHighGoal


        // After shooting into high goal, go park
        val trajectoryFromShootHighGoalToPark: Trajectory=
                trajectoryBuilder(SHOOT_HIGHGOAL, 0.0.toRadians)
                        .lineToLinearHeading(PARK)
                        .build()
        this.trajectoryFromShootHighGoalToPark = trajectoryFromShootHighGoalToPark

        /*
           5 new trajectories for doing high goal and optionally rings pickup
            trajectoryCenterStartToHighGoal
            trajectoryHighGoalToRingAlign
            trajectoryRingAlignToRingGrab
            trajectoryRingGrabToShootHighGoal
            trajectoryHighGoalToWobbleDropoffDeep
         */

        // Start Center to HighGoal shoot position
        // Calculate x position for smooth turn around rings
        val xPositionWeighted = (2.0 * RINGS_ACTUAL.x + 1.0 * SHOOT_HIGHGOAL.x) / 3.0
        val ringLeftToHighGoal = Pose2d(xPositionWeighted, START_CENTER.y, 0.0.toRadians);
        val trajectoryCenterStartToHighGoal: Trajectory=
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
                        .splineTo(ringLeftToHighGoal.vec(), 0.0.toRadians)
                        .splineToLinearHeading(SHOOT_HIGHGOAL, (-80.0).toRadians) // Approach direction
//                        .splineTo(ringLeftToHighGoal)
//                        .(SHOOT_HIGHGOAL)
//                        .lineToLinearHeading(SHOOT_HIGHGOAL)
                        .build()
        this.trajectoryCenterStartToHighGoal = trajectoryCenterStartToHighGoal


        // High Goal to Ring Align
        val trajectoryHighGoalToRingAlign: Trajectory=
                trajectoryBuilder(SHOOT_HIGHGOAL, 180.0.toRadians)
                        .lineToLinearHeading(ringPickupAlign)
                        .build();
        this.trajectoryHighGoalToRingAlign = trajectoryHighGoalToRingAlign

        // High Goal to Wobble Drop (NOT created elsewhere)
        val trajectoryHighGoalToWobbleDropoffDeep: Trajectory=
                when(ZONE_CENTER_VARIABLE) {
                    ZONE_A_CENTER ->
                        trajectoryBuilder(trajectoryRingGrabToShootHighGoal.end(), (-60.0).toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
//                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0,6.0,0.0)),Math.toRadians(-90.0))
//                                .lineToConstantHeading(wobbleDropoffDeep.vec())
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0, 6.0, 1.0.toRadians)), Math.toRadians(-90.0))
                                .lineToSplineHeading(wobbleDropoffDeep)
                                .build();
                    ZONE_B_CENTER ->
                        trajectoryBuilder(trajectoryRingGrabToShootHighGoal.end(), 30.0.toRadians)
//                                .lineToLinearHeading(wobbleDropoffDeep)
                                .splineToSplineHeading(wobbleDropoffAlign.plus(zoneBDeepOffset), 0.0)
                                .lineToConstantHeading(wobbleDropoffDeep.plus(zoneBDeepOffset).vec())
                                .build();
                    else -> // Zone C
                        trajectoryBuilder(trajectoryRingGrabToShootHighGoal.end(), (-20.0).toRadians)
                                // SIMPLE OPTION - turns wrong way near the wall
//                                .lineToLinearHeading(wobbleDropoffDeep)
                                // FANCY OPTION - turns away from the wall
                                .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-20.0, 3.0, 1.0.toRadians)), (-20.0).toRadians)
                                .splineToSplineHeading(wobbleDropoffDeep, 0.0.toRadians)
                                .build();
                }
        this.trajectoryHighGoalToWobbleDropoffDeep = trajectoryHighGoalToWobbleDropoffDeep

        val trajectoryCenterStartToPark: Trajectory=
                trajectoryBuilder(START_CENTER, 0.0.toRadians)
//                        .splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                        .lineToConstantHeading(FAR_PARK.vec())
                        .build()
        this.trajectoryCenterStartToPark = trajectoryCenterStartToPark
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