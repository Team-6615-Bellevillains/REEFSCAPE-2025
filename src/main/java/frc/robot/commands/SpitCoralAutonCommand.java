package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArmSubsystem;

public class SpitCoralAutonCommand extends Command{
    
    private double startingRotations;
    private static final double endRotations = Centimeters.of(30).div(Centimeters.of(6).times(Math.PI)).magnitude();
    private PivotArmSubsystem pivotArmSubsystem;
    
    public SpitCoralAutonCommand(PivotArmSubsystem subsystem){
        this.pivotArmSubsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startingRotations = pivotArmSubsystem.grabberMotorRotations();
        pivotArmSubsystem.setGrabberMotor(0.3);
    }

    @Override
    public boolean isFinished() {
        return pivotArmSubsystem.grabberMotorRotations() - startingRotations >= endRotations;
    }

    @Override
    public void end(boolean interrupted) {
        pivotArmSubsystem.stopMotors();
    }


}
