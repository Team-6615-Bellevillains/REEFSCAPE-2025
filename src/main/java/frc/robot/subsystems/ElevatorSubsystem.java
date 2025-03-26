package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(34, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
    private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
    private static final double downLimit = 0.45;
    private static final double upLimit =  0.55; 
    private static final double elevatorHeight = 57;
    private static final double rotationLimit = 46.125;
    private Position position;
    private static final double l2Inches = 15.25;
    private static final double l3Inches = 30.75;
    private static final double l4Inches = elevatorHeight+0.5;
    private static final double a1Inches = l2Inches-8;
    private static final double a2Inches = l3Inches-8;
    private static final double abInches = elevatorHeight+1;

    public ElevatorSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        rightMotor.getEncoder().setPosition(0);
        leftMotor.getEncoder().setPosition(0);
        config.closedLoop
        .p(0.2)
        .i(0)
        .d(0.15)
        .outputRange(-upLimit, downLimit);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.closedLoop.outputRange(-downLimit, upLimit);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        position = Position.L1;
    }

    @Override
    public void periodic() {
        // if (SharedState.get().getLaserCanDistance() < 50){
        //     Command currentCommand = this.getCurrentCommand();
        //     if (currentCommand != null){
        //         currentCommand.cancel();
        //     }
        //     System.out.println("Can distance <50 (" + SharedState.get().getLaserCanDistance() + ")");
        //     this.setPosition(Position.L1);
        // }

        SmartDashboard.putNumber("Elevator Inches", getPositionInches());
    }

    public static double inchesToRotations(double inches){
        return (rotationLimit/elevatorHeight)*inches;
    }

    public double getPositionInches(){
        return (leftMotor.getEncoder().getPosition())/(rotationLimit/elevatorHeight);
    }

    public double getEncoderPosition(){
        return leftMotor.getEncoder().getPosition();
    }

    private void moveElevator(double inches){
        System.out.println("Moving elevator to "+inches);
        rightController.setReference(-inchesToRotations(inches), ControlType.kPosition);
        leftController.setReference(inchesToRotations(inches), ControlType.kPosition);
    }

    public void setPosition(Position newPosition){
        switch(newPosition){
            case L1:
                moveElevator(0);
                position = Position.L1;
                break;
            case L2:
                moveElevator(l2Inches);
                position = Position.L2;
                break;
            case L3:
                moveElevator(l3Inches);
                position = Position.L3;
                break;
            case L4:
                moveElevator(l4Inches);  
                position = Position.L4;
                break;
            case A1:
                moveElevator(a1Inches);
                position = Position.A1;
                break;
            case A2:
                moveElevator(a2Inches);
                position = Position.A2;
                break;
            case AB:
                moveElevator(abInches);
                position = Position.AB;
                break;
        }
    }

    public boolean atPosition(){
        switch (position) {
            case L1:
                if (getPositionInches()<1.0){
                    return true;
                } else return false;
            case L2:
                if (getPositionInches()<(l2Inches+1) && getPositionInches()>(l2Inches-1)){
                    return true;
                } else return false;

            case L3:
                if (getPositionInches()<(l3Inches+1) && getPositionInches()>(l3Inches-1)){
                    return true;
                } else return false;
            case L4:
                if (getPositionInches()<(l4Inches+1) && getPositionInches()>(l4Inches-1)){
                    return true;
                } else return false;
            case A1:
                if (getPositionInches()<(a1Inches+1) && getPositionInches()>(a1Inches-1)){
                    return true;
                } else return false;
            case A2:
                if (getPositionInches()<(a1Inches+1) && getPositionInches()>(a2Inches-1)){
                    return true;
                } else return false;
            case AB:
                if (getPositionInches()<(abInches+1) && getPositionInches()>(abInches-1)){
                    return true;
                } else return false;
            default:
                return false;
                
        }
    }

    public Position getPosition(){
        return position;
    }
    
    //public Command setPositionCommand(Position level){
    //    return this.runOnce(() -> setPosition(level));
    //}

    public enum Position{
        // coral positions
        L1,
        L2,
        L3,
        L4,
        // algae positions
        A1,
        A2,
        // algae barge
        AB
    }
}
