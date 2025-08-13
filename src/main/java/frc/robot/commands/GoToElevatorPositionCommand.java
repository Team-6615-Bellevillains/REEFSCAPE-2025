package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;

public class GoToElevatorPositionCommand extends Command{
    ElevatorSubsystem elevator;
    PivotArmSubsystem pivot;
    ElevatorSubsystem.Position position;

    public GoToElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, PivotArmSubsystem pivotArmSubsystem, ElevatorSubsystem.Position position){
        this.addRequirements(elevatorSubsystem, pivotArmSubsystem);

        this.elevator = elevatorSubsystem;
        this.pivot = pivotArmSubsystem;
        this.position = position;
    }
    
    @Override
    public void initialize() {
        if(elevator.getPosition() != position){
            if(position == Position.AB){
                pivot.setArmPosition(0);
            } else pivot.setArmPosition(2);
            elevator.setPosition(position);
        }
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("elevatgor.isFinished", elevator.atPosition());
        return elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        switch (position) {
            case L1:
                pivot.setArmPosition(0);
                break;

            case A1:
                pivot.setArmPosition(2);
                pivot.setGrabberMotor(0.2);
                break;

            case A2:
                pivot.setArmPosition(2);
                pivot.setGrabberMotor(0.2);
                break;

            case AB:
                pivot.setArmPosition(0);
                break;

            default:
                pivot.setArmPosition(2);
                break;
        }
    }
}
