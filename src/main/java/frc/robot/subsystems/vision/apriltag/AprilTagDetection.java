package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagDetection {
    private final int fiducialID;
    private final Pose2d robotPose;
    private final Pose2d targetPose;
    private final double ambiguity;

    public AprilTagDetection(int fiducialID, Pose2d robotPose, Pose2d targetPose, double ambiguity) {
        this.fiducialID = fiducialID;
        this.robotPose = robotPose;
        this.targetPose = targetPose;
        this.ambiguity = ambiguity;
    }

    public int getFiducialID() {
        return fiducialID;
    }

    public Pose2d getRobotInFieldPose() {
        return robotPose;
    }

    public Pose2d getRobotToTargetPose() {
        return targetPose;
    }

    public double getAmbiguity() {
        return ambiguity;
    }
}
