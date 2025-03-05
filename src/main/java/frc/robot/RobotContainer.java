package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GoToElevatorPositionCommand;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;
import swervelib.SwerveInputStream;

public class RobotContainer {
    
    private SwerveSubsystem swerve = new SwerveSubsystem();
    private AlgaeGrabberSubsystem algae = new AlgaeGrabberSubsystem();
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private PivotArmSubsystem pivot = new PivotArmSubsystem();
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);
    
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
        configureBindings();
    }

    private void configureBindings(){
        swerve.setDefaultCommand(swerve.driveCommand(driveAngularVelocity, driveAngularVelocitySlow, driverController));
        driverController.a().onTrue(swerve.resetHeading());
        driverController.x().onTrue(algae.resetAlgaeState());
        driverController.leftBumper().onTrue(algae.spitAlgae());
        driverController.rightBumper().onTrue(new GrabAlgaeCommand(algae));

        operatorController.a().whileTrue(pivot.spitCoral());
        operatorController.b().onTrue(pivot.invertInOut());
        operatorController.x().whileTrue(pivot.reverseCoral());
        operatorController.povUp().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L4));
        operatorController.povRight().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L3));
        operatorController.povLeft().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L2));
        operatorController.povDown().onTrue(new GoToElevatorPositionCommand(elevator, pivot, Position.L1));
        
    }

    public Command getAutonomousCommand(){
        return swerve.getAutonomousCommand();
    }
}
