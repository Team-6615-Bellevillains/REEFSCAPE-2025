package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
    
    public RobotContainer(){
        configureBindings();
    }

    private void configureBindings(){
        swerve.setDefaultCommand(swerve.driveCommand(driveAngularVelocity, driverController));
        driverController.a().onTrue(new GrabAlgaeCommand(algae));
        driverController.b().whileTrue(algae.spitAlgae());
        
        operatorController.a().onTrue(pivot.setArmPositionCommand(0));
        operatorController.b().onTrue(pivot.setArmPositionCommand(1));
        operatorController.x().onTrue(pivot.setArmPositionCommand(2));
        operatorController.y().whileTrue(pivot.spitCoral());
        operatorController.povUp().onTrue(elevator.setPositionCommand(4));
        operatorController.povRight().onTrue(elevator.setPositionCommand(3));
        operatorController.povLeft().onTrue(elevator.setPositionCommand(2));
        operatorController.povDown().onTrue(elevator.setPositionCommand(1));
    }
}
