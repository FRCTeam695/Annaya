package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
//import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/*
 * MOTOR SUBSYSTEM:
 *  - this subsystem has everything to do with motors
 *     - includes drivetrains: tank, drive, and possibly swerve
 *     - PID controls for a closed loop motor, published on preferences which is able to change the PID values for the motor through Shuffleboard
 *     - work with limit switches, both mechanical and magnetic, which start / stop the motor based on the presence of one of these switches
 */

public class MotorSubsystem extends SubsystemBase {

  //drive motors
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollower;
  private CANSparkMax leftLeader;
  private CANSparkMax leftFollower;

  //differential drive object
  private final DifferentialDrive robotDrive;

  private DigitalInput lSwitch;
  private DigitalInput mechSwitch;

  public CANSparkFlex motor;
  private RelativeEncoder myEncoder;
  public SparkPIDController pid;

  public double kP = Constants.PID_constants.kP;
  public double kI = Constants.PID_constants.kI;
  public double kD = Constants.PID_constants.kD;
  public double kFF = Constants.PID_constants.kFF;
  public double setPoint = Constants.PID_constants.setPointRPM;

  public MotorSubsystem() {
    //CAN SparkMax motors: Arcade Drive (as well as Tank Drive, which is driven using two joysticks' y axes)
     rightLeader = new CANSparkMax(11, MotorType.kBrushless); //all these are changed to CAN IDs of the motors on the chassee
     rightFollower = new CANSparkMax(13, MotorType.kBrushless);
     leftLeader = new CANSparkMax(12, MotorType.kBrushless);
     leftFollower = new CANSparkMax(10, MotorType.kBrushless);

     rightLeader.restoreFactoryDefaults();
     leftLeader.restoreFactoryDefaults();
     rightFollower.restoreFactoryDefaults();
     leftFollower.restoreFactoryDefaults();
     //rightLeader.setInverted(true);
     //rightFollower.setInverted(true);
     //leftLeader.setInverted(true);
     //leftFollower.setInverted(true);

     //the behavior of rightFollower and leftFollower motors will mimic those of rightLeader and leftLeader respectively
     rightFollower.follow(rightLeader);
     leftFollower.follow(leftLeader);

     robotDrive = new DifferentialDrive(leftLeader, rightLeader);

     lSwitch = new DigitalInput(9);
     mechSwitch = new DigitalInput(8);
     motor = new CANSparkFlex(53, MotorType.kBrushless);
     motor.restoreFactoryDefaults();

     pid = motor.getPIDController();
     //pid = new PIDController(Constants.PID_constants.kP, Constants.PID_constants.kI, Constants.PID_constants.kD);

     pid.setP(kP);
     pid.setI(kI);
     pid.setD(kD);
     pid.setOutputRange(Constants.PID_constants.kMinOutput, Constants.PID_constants.kMaxOutput);
     pid.setFF(kFF);

     myEncoder = motor.getEncoder();

     Preferences.initDouble(Constants.PID_constants.kP_value, kP);
     Preferences.initDouble(Constants.PID_constants.kI_value, kI);
     Preferences.initDouble(Constants.PID_constants.kD_value, kD);
     Preferences.initDouble(Constants.PID_constants.kFF_value, kFF);
     Preferences.initDouble(Constants.PID_constants.kSetPoint_value, setPoint);
  }

  /* DEFAULT COMMANDS (RUNNABLE COMMANDS) FOR ARCADE DRIVE AND TANK DRIVE ARE BELOW */


  //below is for arcade drive - accessed using x and y sticks on command xbox controller

  public Command arcadeDriveCommand(DoubleSupplier stickSpeed, DoubleSupplier stickRotation) {
    return run (

      () -> {
        double speed = stickSpeed.getAsDouble();
        double rotation = stickRotation.getAsDouble();

        robotDrive.arcadeDrive(speed, rotation);
      }
    );
  }

  //below is for tank drive - accessed using two joysticks visible in robot container -> left joystick's y axis is used for left speed, and right joystick's y axis is used for right speed

  public Command tankDriveCommand(DoubleSupplier leftVal, DoubleSupplier rightVal) {
    return new RunCommand (
        () -> {
            double leftSpeed = leftVal.getAsDouble();
            double rightSpeed = rightVal.getAsDouble();
    
            SmartDashboard.putNumber("Left Speed: ", leftSpeed);
            SmartDashboard.putNumber("Right Speed: ", rightSpeed);
            robotDrive.tankDrive(leftSpeed, rightSpeed);
          }
        ,this
        );
      }

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              /* one-time action goes here */
            });
      }

  public Command arcadeDriveJoystickCommand(DoubleSupplier leftJSpeed, DoubleSupplier rightJRotation) {
    return new RunCommand (
        () -> {
            double speed = leftJSpeed.getAsDouble();
            double rotation = rightJRotation.getAsDouble();

            robotDrive.arcadeDrive(speed, rotation);
        }
        ,this
    );
  }

  public Command magneticSwitchCommand() {
    return new FunctionalCommand(

      () -> switchInit(),

      () -> {
        if(lSwitch.get() == false) {
            motor.set(0);
        }
        else {
            motor.set(0.06);
        }
      },

      interrupted -> switchInt(),

      () -> endCommand(),

      this);
  }

  public void switchInit() {

  }

  public void switchInt() {
    motor.set(0);
  }

  public boolean endCommand() {
    return false;
  }

  public Command mechSwitchCommand() {
    return new FunctionalCommand(

      () -> mechInit(),

      () -> {
        if(mechSwitch.get() == true) {
            motor.set(0.06);
        }
        else {
            motor.set(0);
        }
      },

      interrupted -> mechInt(),

      () -> mechEndCommand(),

      this);
  }

  public void mechInit() {

  }

  public void mechInt() {
    motor.set(0);
  }

  public boolean mechEndCommand() {
    return false;
  }



  public Command MotorPIDCommand() {
    return new FunctionalCommand(
      () -> motorInit(),

      () -> {

        kP = Preferences.getDouble(Constants.PID_constants.kP_value, kP);
        pid.setP(kP);
        kI = Preferences.getDouble(Constants.PID_constants.kI_value, kI);
        pid.setI(kI);
        kD = Preferences.getDouble(Constants.PID_constants.kD_value, kD);
        pid.setD(kD);
        kFF = Preferences.getDouble(Constants.PID_constants.kFF_value, kFF);
        pid.setFF(kFF);
        setPoint = Preferences.getDouble(Constants.PID_constants.kSetPoint_value, setPoint);
    

        pid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        System.out.println("The motor is running");
        //motor.set(pid.calculate(myEncoder.getVelocity(), Constants.PID_constants.setPoint));

        //SmartDashboard.putNumber("Set Point: ", 0.1 * Constants.PID_constants.maxRPM);
        //pid.setTolerance(0, 0.1);
        //pid.atSetpoint();

        /*
        SmartDashboard.putNumber("kP", Constants.PID_constants.kP);
        SmartDashboard.putNumber("kI", Constants.PID_constants.kI);
        SmartDashboard.putNumber("kD", Constants.PID_constants.kD);
        SmartDashboard.putNumber("Set point: ", Constants.PID_constants.setPointRPM);
        */
        System.out.println("Motor speed is " + myEncoder.getVelocity());
        SmartDashboard.putNumber("Motor value: ", myEncoder.getVelocity());
      },

      interrupted -> {
        //pid.setReference(0, CANSparkMax.ControlType.kVelocity);
      },

      () -> motorEndCondition(),

      this);
  }

  private void motorInit() {
    SmartDashboard.putNumber("Set point velocity in RPM: ", 1000);
  }

  private boolean motorEndCondition() {
    return false;
  }




  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean motorSystemCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}