package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AprilTagDataUtil;
import frc.robot.utils.AutoAlignUtil;

@Logged
public class AlgaeAlignAssist extends Command {
    private final PIDController leftRightController = new PIDController(5.0, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(5.0, 0, 0, 
        new TrapezoidProfile.Constraints(
            DegreesPerSecond.of(540).in(RadiansPerSecond), 
            DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond)
        )
    );

    private Pose2d targetAlgaePose;

    private final Supplier<Double> joystickX;

    private final boolean shouldFinishWhenAtSetpoint;
    private double offsetInches;
    private final double velocityMultiplier = 2.5;

    private static final Distance LEFT_RIGHT_POSITION_TOLERANCE = Inches.of(0.25);
    private static final Angle ROTATION_TOLERANCE = Degrees.of(1);

    private final SwerveSubsystem swerveSubsystem;
    
    public AlgaeAlignAssist(SwerveSubsystem swerveSubsystem, Supplier<Double> joystickX, Supplier<Double> joystickY, boolean shouldFinishWhenAtSetpoint, Target target) {
        addRequirements(swerveSubsystem);

        this.joystickX = joystickX;

        this.shouldFinishWhenAtSetpoint = shouldFinishWhenAtSetpoint;

        switch (target) {
            case ALGAE:
                offsetInches = 2.25; 
                break;
        
            case LEFT:
                offsetInches = 6.5;
                break;

            case RIGHT:
                offsetInches = -6.5;
                break;
        }

        if (shouldFinishWhenAtSetpoint) {
            leftRightController.setTolerance(LEFT_RIGHT_POSITION_TOLERANCE.in(Meters));
            thetaController.setTolerance(ROTATION_TOLERANCE.in(Radians));
        }

        thetaController.enableContinuousInput(0, Units.degreesToRadians(360));

        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        leftRightController.reset();
        thetaController.reset(
            swerveSubsystem.getPose().getRotation().getRadians(), 
            swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond
        );

        targetAlgaePose = AutoAlignUtil.getClosestAprilTagPose(swerveSubsystem.getPose());
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerveSubsystem.getPose();

        // Pretend as if target algae is at the back of the reef and rotate the robot and target accordingly
        // Makes for easy calculation of left/right error
        Pose2d normalizedRobotPose = normalizeFromReef(robotPose, targetAlgaePose);
        Pose2d normalizedTargetPose = normalizeFromReef(targetAlgaePose, targetAlgaePose);

        double leftRightVelocity = leftRightController.calculate(normalizedRobotPose.getY(), normalizedTargetPose.getY() + Meters.convertFrom(offsetInches, Inches));
        double thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), targetAlgaePose.getRotation().getRadians());

        double forwardsBackwardsVelocity = -joystickX.get() * velocityMultiplier;

        swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(
            forwardsBackwardsVelocity,
            leftRightVelocity,
            thetaVelocity
        ));
    }

    @Override
    public boolean isFinished() {
        return shouldFinishWhenAtSetpoint && leftRightController.atSetpoint() && thetaController.atGoal();
    }

    Pose2d normalizeFromReef(Pose2d in, Pose2d tag) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d centerOfReef = AprilTagDataUtil.getReefCenter(alliance);

        return in.rotateAround(centerOfReef, tag.getRotation().unaryMinus());
    }
     
    public enum Target{
        ALGAE,
        LEFT,
        RIGHT
    }
}
