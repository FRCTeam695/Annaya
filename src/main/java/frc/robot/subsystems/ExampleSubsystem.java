// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ExampleSubsystem extends SubsystemBase {
  DoublePublisher xPub;
  DoublePublisher yPub;
  
  BooleanPublisher xButton;
  DoublePublisher xStick;

  AddressableLED led;
  AddressableLEDBuffer buffer;
  Servo servo;
  CommandXboxController controller;

  //private CANSparkFlex motor;
  //arcade drive motors
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollower;
  private CANSparkMax leftLeader;
  private CANSparkMax leftFollower;

  //differential drive object
  //private final DifferentialDrive robotDrive;

  private double x = 0;
  private double y = 0;

  public int maxCnt = 50;
  public int count = 0;

  DigitalInput lSwitch;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem(CommandXboxController controller) {
    // Get the default instance of NetworkTables that was created automatically
    // when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    this.controller = controller;

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("datatable");

    // Start publishing topics within that table that correspond to the X and Y values
    // for some operation in your program.
    // The topic names are actually "/datatable/x" and "/datatable/y".
    xPub = table.getDoubleTopic("x").publish();
    yPub = table.getDoubleTopic("y").publish();
    xStick = table.getDoubleTopic("XStick").publish();
    xButton = table.getBooleanTopic("xButton").publish();
    servo = new Servo(0);
    led = new AddressableLED(1);
     buffer = new AddressableLEDBuffer(24);
     led.setLength(buffer.getLength());
     for(int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
     led.setData(buffer);
     led.start();

    //CAN SparkFlex motors: Arcade Drive (as well as Tank Drive, which is driven using a joystick)
     rightLeader = new CANSparkMax(11, MotorType.kBrushless); //changed to CAN IDs of the motors on the chassee
     rightFollower = new CANSparkMax(13, MotorType.kBrushless);
     leftLeader = new CANSparkMax(12, MotorType.kBrushless);
     leftFollower = new CANSparkMax(10, MotorType.kBrushless);

     rightLeader.setInverted(true);
     rightFollower.setInverted(true);
     leftLeader.setInverted(true);
     leftFollower.setInverted(true);

     rightFollower.follow(rightLeader);
     leftFollower.follow(leftLeader);

     //robotDrive = new DifferentialDrive(leftLeader, rightLeader);

     lSwitch = new DigitalInput(9);
     //motor = new CANSparkFlex(53, MotorType.kBrushless);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  public Command LEDCommand()
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> testInit1(),
     
      // ** EXECUTE
      ()-> testExecute1(),
      /*above takes targetvalue as raw axis in RobotContainer (see that), and will keep updating it using get as double method as
      execute gets called multiple times. see comment in execute*/

      // ** ON INTERRUPTED
      interrupted-> testInterrupt1(),
     
      // ** END CONDITION
      ()-> testEndCondition1(),


      // ** REQUIREMENTS
      this);


  }


  private void testInit1()
  {
    //
  }


  private void testExecute1()
  {
    /* value parameter are the values from get as double method in command method. set method below changes xStick value in network table
    to the value variable since set method is part of DoublePublisher class */
    //xStick.set(value);
    //maxCnt = (int) (50 - Math.abs(value) * 50);
    //count = 0;
    if(controller.getHID().getXButton()) {
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 0, 255);
        //servo.setSpeed(servo.getSpeed() + 0.05);
      }
      led.setData(buffer);
      //servo.setSpeed(0);
    }
    else if(controller.getHID().getYButton()) {
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 255, 255, 0);
      }
      led.setData(buffer);
    }
    else if(controller.getHID().getBButton()) {
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 255, 0, 0);
        //servo.setSpeed(servo.getSpeed() + 0.05);
      }
      led.setData(buffer);
      //servo.setSpeed(0);
    }
    else if(controller.getHID().getAButton()) {
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 255, 0);
        //servo.setSpeed(servo.getSpeed() + 0.05);
      }
      led.setData(buffer);
      //servo.setSpeed(0);
    }
  }


  private void testInterrupt1()
  {
    //led.setData(buffer);
    //servo.setSpeed(0);
  }


  private boolean testEndCondition1()
  {
    return(false);
  }


//servo command
public Command ServoCommand(int direction)
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> testServoInit(),
     
      // ** EXECUTE
      ()-> testServoExecute(direction),

      // ** ON INTERRUPTED
      interrupted-> testServoInterrupted1(),
     
      // ** END CONDITION
      ()-> testServoEndCondition(),

      // ** REQUIREMENTS
      this);
  }

  private void testServoInit() {
    //servo.set(0.5);
  }

  private void testServoExecute(int direction) {
    
    if(direction == 1) {
      //for(int i = 0; i < 5; i++) {
        servo.set(0.6);
      //}
    }
    if(direction == 2) {
      //for(int i = 0; i < 5; i++) {
        servo.set(0.4);
      //}
    } 
  }
  
  private void testServoInterrupted1() {
    servo.set(0.5);
  }

  private boolean testServoEndCondition() {
    return (false);
  }
  

public Command stickCommand(DoubleSupplier targetvalue)
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> stickInit(),
     
      // ** EXECUTE
      ()-> {
        double value = targetvalue.getAsDouble();
        System.out.println(value);
        servo.set((value / 2) + 0.5);
      },

      // ** ON INTERRUPTED
      interrupted-> stickInterrupted1(),
     
      // ** END CONDITION
      ()-> stickEndCondition(),

      // ** REQUIREMENTS
      this);
  }

  private void stickInit() {
    //
  }
  
  private void stickInterrupted1() {
    servo.set(0.5);
  }

  private boolean stickEndCondition() {
    return (false);
  }



  public Command LEDStickCommand(DoubleSupplier targetvalue) //this is for changing the dimness of the LEDs (brightness)
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> stickInit(),
     
      // ** EXECUTE
      ()-> {
        double value = targetvalue.getAsDouble();
        System.out.println(value);
        for(int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 0, (int)(Math.abs(value) * 255), 0);
        }
        led.setData(buffer);
      },

      // ** ON INTERRUPTED
      interrupted-> stickInterrupted1(),
     
      // ** END CONDITION
      ()-> stickEndCondition(),

      // ** REQUIREMENTS
      this);
  }

  public Command MotorCommand(DoubleSupplier target) {
    return new FunctionalCommand(
      () -> motorInit(),

      () -> {
        double value = target.getAsDouble();
        System.out.println(value);
        rightLeader.set(value);
      },

      interrupted -> motorInterrupted(),

      () -> motorEndCondition(),

      this);
  }

  private void motorInit() {
    rightLeader.set(0);
  }

  private void motorInterrupted() {
    rightLeader.set(0);
  }

  private boolean motorEndCondition() {
    return false;
  }

  /* 
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
  */

  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
      xPub.set(x);
      yPub.set(y);
      x += 0.05;
      y += 1.0;
  }
}

//HOMEWORK for july 15: take game controller home
//import game controller library
//create game controller object
//find designated method that reads what button you want
//do that conditionally in periodic method: the datatable values should only change when that designated button is being pressed