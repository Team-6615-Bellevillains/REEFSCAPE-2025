package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.GoToElevatorPositionCommand;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.commands.LoadCoralCommand;
import frc.robot.commands.LoadCoralLaserCANCommand;
import frc.robot.commands.ShootBargeCommand;
import frc.robot.commands.SpitCoralAutonCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;
import frc.robot.utils.AprilTagDataUtil;
import frc.robot.utils.AutoAlignUtil;
import frc.robot.utils.AutoAlignUtil.CoralScoreDirection;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    private SwerveSubsystem swerve = new SwerveSubsystem(driverController);
    private AlgaeGrabberSubsystem algae = new AlgaeGrabberSubsystem();
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private PivotArmSubsystem pivot = new PivotArmSubsystem();
    private LEDSubsystem ledSubsystem = new LEDSubsystem();

    private SendableChooser<Command> autoChooser;    
    
    // from the YAGSL example project, hence the diabolical formatting
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(()->{
                                                                return -driverController.getRightX();
                                                            })
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -0.5,
                                                                () -> driverController.getLeftX() * -0.5)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(0.05)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true).cubeRotationControllerAxis(true).cubeTranslationControllerAxis(true);


    public RobotContainer(){
        // Robot is always loaded at start of Autonomous
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> SharedState.get().setLoaded(true)));

        NamedCommands.registerCommand("elevatorL1", new GoToElevatorPositionCommand(elevator, pivot, Position.L1));
        NamedCommands.registerCommand("elevatorL2", new GoToElevatorPositionCommand(elevator, pivot, Position.L2));
        NamedCommands.registerCommand("elevatorL3", new GoToElevatorPositionCommand(elevator, pivot, Position.L3));
        NamedCommands.registerCommand("elevatorL4", new GoToElevatorPositionCommand(elevator, pivot, Position.L4));

        NamedCommands.registerCommand("autoAlignLeft", AutoAlignUtil.autoAlign(swerve, CoralScoreDirection.LEFT));
        NamedCommands.registerCommand("autoAlignRight", AutoAlignUtil.autoAlign(swerve, CoralScoreDirection.RIGHT));
        
        NamedCommands.registerCommand("loadCoralAuto", new LoadCoralLaserCANCommand(pivot));
        NamedCommands.registerCommand("spitCoralAuto", new SpitCoralAutonCommand(pivot));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Fixes certain PathPlanner operations being slow when first ran by simulating a path
        // Does **not** move the robot

        configureBindings();

        List<Pose2d> coralTagPoses = AprilTagDataUtil.get().getCoralAprilTagPoses(DriverStation.Alliance.Blue);
        for (int i = 0; i < 6; ++i) {
            Pose2d leftBranchPose = AutoAlignUtil.offsetAprilTagPose(coralTagPoses.get(i), CoralScoreDirection.LEFT);
            Pose2d rightBranchPose = AutoAlignUtil.offsetAprilTagPose(coralTagPoses.get(i), CoralScoreDirection.RIGHT);

            System.out.printf("Tag %d Left Position: %s\nRight Position: %s\n", i + 17, leftBranchPose,
                    rightBranchPose);
        }
    }

    private void configureBindings(){
        swerve.setDefaultCommand(swerve.driveCommand(driveAngularVelocity, driveAngularVelocitySlow, driverController));
        driverController.a().onTrue(swerve.resetHeading());
        driverController.x().onTrue(algae.resetAlgaeState());
        driverController.leftBumper().whileTrue(algae.spitAlgae());
        driverController.rightBumper().onTrue(new GrabAlgaeCommand(algae));
        driverController.start().onTrue(Commands.print(swerve.getPose().toString()));

        driverController.povLeft().onTrue(AutoAlignUtil.autoAlign(swerve, CoralScoreDirection.LEFT));
        driverController.povRight().onTrue(AutoAlignUtil.autoAlign(swerve, CoralScoreDirection.RIGHT));

        operatorController.a().whileTrue(pivot.spitCoral());
        operatorController.b().onTrue(pivot.invertInOut());
        operatorController.x().whileTrue(pivot.reverseCoral());
        operatorController.povUp().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L3));
        operatorController.povRight().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L4));
        operatorController.povLeft().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L2));
        operatorController.povDown().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L1));
        operatorController.rightBumper().onTrue(new LoadCoralLaserCANCommand(pivot));
        operatorController.y().onTrue(pivot.resetCoralAngleCommand());
        operatorController.leftBumper().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.A1));
        operatorController.leftTrigger().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.A2));
        operatorController.rightTrigger().onTrue(pivot.grabAlgaeBargeShotCommand());
        operatorController.back().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.AB));
        operatorController.start().whileTrue(pivot.throwBallCommand());
    }

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}
