package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(34, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

    private static final Dimensionless DOWNWARDS_OUTPUT_LIMIT = Percent.of(45);
    private static final Dimensionless UPWARDS_OUTPUT_LIMIT = Percent.of(55); 
    private static final Distance ELEVATOR_HEIGHT = Inches.of(57);
    private static final Angle MOTOR_ROTATION_LIMIT = Rotations.of(46.125);

    private static final Distance L1_POSITION = Inches.of(0.5);
    private static final Distance L2_POSITION = Inches.of(15.25);
    private static final Distance L3_POSITION = Inches.of(30.75);
    private static final Distance L4_POSITION = ELEVATOR_HEIGHT.plus(Inches.of(0.5));
    private static final Distance A1_POSITION = L2_POSITION.minus(Inches.of(8));
    private static final Distance A2_POSITION = L3_POSITION.minus(Inches.of(8));
    private static final Distance AB_POSITION = ELEVATOR_HEIGHT.plus(Inches.of(1)).plus(Inches.of(1));

    private static final Distance POSITION_TOLERANCE = Inches.of(1);

    private SetpointID currentSetpointID;

    public ElevatorSubsystem(){
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        leaderConfig.closedLoop
            .p(0.2)
            .i(0)
            .d(0.15)
            .outputRange(UPWARDS_OUTPUT_LIMIT.unaryMinus().magnitude(), DOWNWARDS_OUTPUT_LIMIT.magnitude());

        rightMotor.getEncoder().setPosition(0);
        rightMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(rightMotor, true);

        leftMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentSetpointID = SetpointID.L1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Inches", getPosition().in(Inches));

        SmartDashboard.putNumber("Left Elevator Output Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Elevator Output Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left Elevator Applied Output", leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Right Elevator Applied Output", rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Left Elevator Temperature", leftMotor.getMotorTemperature());
        SmartDashboard.putNumber("Right Elevator Temperature", rightMotor.getMotorTemperature());
    }

    public static Angle positionToMotorRotations(Distance inches){
        return inches.timesConversionFactor(MOTOR_ROTATION_LIMIT.div(ELEVATOR_HEIGHT));
    }

    public Distance getPosition(){
        return Rotations.of(leftMotor.getEncoder().getPosition())
                        .unaryMinus()
                        .timesConversionFactor(ELEVATOR_HEIGHT.div(MOTOR_ROTATION_LIMIT));
    }

    private static Map<SetpointID, Distance> setpointIDToDistance = new HashMap<SetpointID, Distance>(){{
        put(SetpointID.L1, L1_POSITION);
        put(SetpointID.L2, L2_POSITION);
        put(SetpointID.L3, L3_POSITION);
        put(SetpointID.L4, L4_POSITION);

        put(SetpointID.A1, A1_POSITION);
        put(SetpointID.A2, A2_POSITION);
        put(SetpointID.AB, AB_POSITION);
    }};

    public void setReference(SetpointID newSetpointID){
        this.currentSetpointID = newSetpointID;
        rightController.setReference(
            positionToMotorRotations(setpointIDToDistance.get(newSetpointID))
                .unaryMinus()
                .magnitude(), 
            ControlType.kPosition
        );
    }

    public boolean atPosition(){
        if (Robot.isSimulation()) {
            return true;
        }

        return getPosition().isNear(setpointIDToDistance.get(currentSetpointID), POSITION_TOLERANCE);
    }

    public SetpointID getCurrentSetpointID(){
        return currentSetpointID;
    }
    
    public enum SetpointID{
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

    public Command zeroElevatorCommand(){
        return this.runOnce(()->{
            //leftMotor.getEncoder().setPosition(0);
            rightMotor.getEncoder().setPosition(0);
        });
    }
}
