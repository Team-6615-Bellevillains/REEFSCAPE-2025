package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;

public class ShootBargeCommand extends Command {
    
    private ElevatorSubsystem elevator;
    private PivotArmSubsystem pivot;
    
    public ShootBargeCommand(ElevatorSubsystem elevator, PivotArmSubsystem pivot){
        this.addRequirements(elevator, pivot);
        this.elevator = elevator;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        elevator.setPosition(Position.L4);
    }

    @Override
    public void execute() {
        if(elevator.getPositionInches() > 56){
            pivot.throwBall();
        }
    }
    
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stopMotors();
    }
}
