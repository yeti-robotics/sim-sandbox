package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;

public class AprilTagSimulator {
    VisionSystemSim visionSim;
    List<AprilTagCamSim> aprilTagCamSims;

    public AprilTagSimulator() {
        visionSim = new VisionSystemSim("main");
        aprilTagCamSims = new ArrayList<>();

        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    public void addCamera(AprilTagCamSim aprilTagCamSim) {
        visionSim.addCamera(aprilTagCamSim.getCameraSim(), aprilTagCamSim.getTransform());
        aprilTagCamSims.add(aprilTagCamSim);
    }

    public void update(Pose2d pose) {
        visionSim.update(pose);
        aprilTagCamSims.forEach(AprilTagCamSim::publishSeenTags);
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
