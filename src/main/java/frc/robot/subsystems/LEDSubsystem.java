package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//This is the main subsystem that will be used for LED triggers and signals, as well as servos

public class LEDSubsystem extends SubsystemBase {
    AddressableLED led;
    AddressableLEDBuffer buffer;
    Servo servo;

    public LEDSubsystem() {
        servo = new Servo(0);
        led = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(24);
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        led.setData(buffer);
        led.start();
    }
    
    //FIRST COMMAND - CONTROLS THE COLOR OF LED'S BASED ON THE COLOR OF THE BUTTON ON THE CONTROLLER THAT IS PRESSED

    public Command LEDCommand() {
        return new FunctionalCommand(
            
        // ** INIT
            ()-> {},
            
            // ** EXECUTE
            ()-> LEDExecute1(),
            /*above takes targetvalue as raw axis in RobotContainer (see that), and will keep updating it using get as double method as
            execute gets called multiple times. see comment in execute*/

            // ** ON INTERRUPTED
            interrupted-> testInterrupt1(),
            
            // ** END CONDITION
            ()-> testEndCondition1(),


            // ** REQUIREMENTS
            this);
    }


  private void LEDExecute1()
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

  /*************/

  //SECOND COMMAND - HAS TO DO WITH SERVOS, AND IT TURNS THE SERVO CLOCKWISE OR COUNTERCLOCKWISE BASED ON THE DIRECTION THAT IS SPECIFIED
  //either 1 or 2 are pressed to control which direction the servo turns

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


    /*************/

//THIRD COMMAND - CONTROLS THE SPEED AND DIRECTION OF THE SERVO BASED ON HOW AN AXIS OF THE CONTROLLER IS MOVED
public Command stickCommand(DoubleSupplier targetvalue)
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> {},
     
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

  
  private void stickInterrupted1() {
    servo.set(0.5);
  }

  private boolean stickEndCondition() {
    return (false);
  }

  /*************/


  //FOURTH COMMAND - CONTROLS THE BRIGHTNESS OF THE LED BASED ON STICK MOVEMENT

  public Command LEDStickCommand(DoubleSupplier targetvalue) //this is for changing the dimness of the LEDs (brightness)
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> {},
     
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean LEDCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


}
