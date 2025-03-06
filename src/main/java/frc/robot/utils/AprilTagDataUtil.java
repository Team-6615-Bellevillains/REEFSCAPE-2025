package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

// This class is a Singleton.
// That means that it can only be instantiated once,
// and that there is easy access to that instance.
//
// Usage: AprilTagDataUtil.get().getFieldLayout();
public class AprilTagDataUtil {
    // TODO: Add "Fudge Factors" i.e., if Tag 3 is off by 15mm at Belleville, we adjust that in the layout.

    private AprilTagFieldLayout aprilTagFieldLayout;

    private static AprilTagDataUtil instance;

    private AprilTagDataUtil() {
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                    "2025-reefscape-welded.json"
                ).toAbsolutePath()
            );
        } catch (IOException e) {
            aprilTagFieldLayout = null;
            e.printStackTrace();
        }
    }

    public static AprilTagDataUtil get() {
        if (instance == null) {
            instance = new AprilTagDataUtil();
        }

        return instance;
    }

    public List<AprilTag> getAprilTags() {
        if (instance.aprilTagFieldLayout == null) {
            return new ArrayList<>();
        }

        return instance.aprilTagFieldLayout.getTags();
    }
}
