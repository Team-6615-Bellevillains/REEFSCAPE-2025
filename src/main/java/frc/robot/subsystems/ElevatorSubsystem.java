package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(34, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
    private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
    private static final double downLimit = 0.15;
    private static final double upLimit = 0.3; 
    private static final double elevatorHeight = 57;
    private static final double rotationLimit = 46.125;

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
    }

    private double inchesToRotations(double inches){
        return (rotationLimit/elevatorHeight)*inches;
    }

    private void moveElevator(double inches){
        rightController.setReference(-inchesToRotations(inches), ControlType.kPosition);
        leftController.setReference(inchesToRotations(inches), ControlType.kPosition);
    }

    public void setPosition(int level){
        switch(level){
            case 1:
                moveElevator(0);
                break;
            case 2:
                moveElevator(19);
                break;
            case 3:
                moveElevator(36);
                break;
            case 4:
                moveElevator(elevatorHeight+0.5);
                break;
        }
    }

    public Command setPositionCommand(int level){
        return this.runOnce(() -> setPosition(level));
    }
}
