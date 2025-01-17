package frc.robot.subsystems.swervedrive.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax rightElevatorMotor = new SparkMax(Constants.ElevatorConstants.rightElevatorMotorID,
            MotorType.kBrushless);
    private final SparkMax leftElevatorMotor = new SparkMax(Constants.ElevatorConstants.leftElevatorMotorID,
            MotorType.kBrushless);

    private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.ELEVATOR_ENCODER_PORT);

    private PIDController PID = new PIDController(Constants.ElevatorConstants.ELEVATOR_KP,
            Constants.ElevatorConstants.ELEVATOR_KI, Constants.ElevatorConstants.ELEVATOR_KD);
    private ElevatorFeedforward FF = new ElevatorFeedforward(Constants.ElevatorConstants.ELEVATOR_KS,
            Constants.ElevatorConstants.ELEVATOR_KG, Constants.ElevatorConstants.ELEVATOR_KV);

    private boolean enabled = false;
/* 
    private TrapezoidProfile profile = new TrapezoidProfile(Constants.ElevatorConstants.kArmMotionConstraint);
    private TrapezoidProfile.State goalState, setPoint;
*/

    public ElevatorSubsystem() {
        // rightElevatorMotor.setInverted(true);

   
        
    }

    public void setVoltage(double voltage) {
        rightElevatorMotor.set(voltage);
        leftElevatorMotor.set(voltage);
    }

    @Override
    public void periodic() {
       
    }

    public void enable() {
        enabled = true;
    }

    public void disabled() {
        enabled = false;
    }

    public double getPosition(){
        return elevatorEncoder.get();
    }

    public boolean isBusy(){
        return !PID.atSetpoint();
    }
    /* 
    public double getVelocity(){
        return elevatorEncoder.getRate();
    }
   */

   public void setPosition(double position){
    //goalState = new TrapezoidProfile.State(position, 0);
   }
} 
