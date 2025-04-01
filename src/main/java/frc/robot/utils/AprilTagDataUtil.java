package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

// This class is a Singleton.
// That means that it can only be instantiated once,
// and that there is easy access to that instance.
//
// Usage: AprilTagDataUtil.get().getFieldLayout();
public class AprilTagDataUtil {
    // TODO: Add "Fudge Factors" i.e., if Tag 3 is off by 15mm at Belleville, we adjust that in the layout.

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private List<AprilTag> redAprilTags = aprilTagFieldLayout.getTags()
                                                            .stream()
                                                            .filter((aprilTag) -> getSide(aprilTag.pose.toPose2d()) == DriverStation.Alliance.Red)
                                                            .toList();
    private List<AprilTag> blueAprilTags = aprilTagFieldLayout.getTags()
                                                            .stream()
                                                            .filter((aprilTag) -> getSide(aprilTag.pose.toPose2d()) == DriverStation.Alliance.Blue)
                                                            .toList();

    private List<Pose2d> redCoralPoses = redAprilTags
                                            .stream()
                                            .filter((aprilTag) -> 6 <= aprilTag.ID && aprilTag.ID <= 11)
                                            .map((aprilTag) -> aprilTag.pose.toPose2d())
                                            .toList();
    private List<Pose2d> blueCoralPoses = blueAprilTags
                                            .stream()
                                            .filter((aprilTag) -> 17 <= aprilTag.ID && aprilTag.ID <= 22)
                                            .map((aprilTag) -> aprilTag.pose.toPose2d())
                                            .toList();


    private static AprilTagDataUtil instance;

    private AprilTagDataUtil() { }

    public static AprilTagDataUtil get() {
        if (instance == null) {
            instance = new AprilTagDataUtil();
        }

        return instance;
    }

    public List<Pose2d> getCoralAprilTagPoses(DriverStation.Alliance alliance) {
        switch (alliance) {
            case Red:
                return redCoralPoses;
            case Blue:
                return blueCoralPoses;
            default:
                return new ArrayList<>();
        }
    }

    public Translation2d getReefCenter(DriverStation.Alliance alliance) {
        int tagIDA, tagIDB;

        switch (alliance) {
            case Red:
                tagIDA = 6;
                tagIDB = 9;
            case Blue:
            default:
                tagIDA = 17;
                tagIDB = 20;
        }

        return aprilTagFieldLayout.getTagPose(tagIDA).get()
                .plus(
                    aprilTagFieldLayout.getTagPose(tagIDB).get()
                        .minus(aprilTagFieldLayout.getTagPose(tagIDA).get())
                        .div(2)
                )
                .getTranslation()
                .toTranslation2d();
    }

    private DriverStation.Alliance getSide(Pose2d pose) {
        return pose.getX() > aprilTagFieldLayout.getFieldLength()/2 ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue;
    } 
}
