package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * SWERVE DRIVE SUBSYSTEM: 
 *   - this subsystem can take a number of swerve drive modules and put them together to operate as one drive
 */

public class SwerveDriveSubsystem extends SubsystemBase {

    //creating the swerve drive module for one corner and initializing its motor IDs
    public final SwerveDriveModule corner = new SwerveDriveModule(13, 12, 11);
    
    //creating the variables for full trackWidth and trackLength
    public double trackWidth;
    public double trackLength;

    public SwerveDriveSubsystem() {
        corner.configCANcoder(1); //change offset as necessary
        
        //initializing these values according to measurement on chassis

        trackWidth = 6; //change as necessary
        trackLength = 6; //also change this as necessary
    }

    //this is a command to constantly take in values for FWD, STR, and RWD based on double suppliers from controller sticks
    public Command drive(DoubleSupplier FWD, DoubleSupplier STR, DoubleSupplier RCW) {
        return new RunCommand(
            () -> {
                //converting double suppliers into double values to be used in SwerveDriveModule method setSpeedAndAngle()
                double fwd = FWD.getAsDouble();
                double str = STR.getAsDouble();
                double rcw = RCW.getAsDouble();

                //setting the swerve drive module's speed and angle according to these values
                corner.setSpeedAndAngle(fwd, str, rcw, trackWidth, trackLength); //converting fwd, str, and rwd to RPM units
                //corner.setSpeedAndAngle(6380 * fwd, 6380 * str, 6380 * rcw, trackWidth, trackLength); //converting fwd, str, and rwd to RPM units
            }, 
        this);
    }

    public Command spin(DoubleSupplier amount) {
        return new RunCommand(
            () -> {
                double a = amount.getAsDouble();
                corner.spin(a);
                
            }, this);
    }

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              /* one-time action goes here */
            });
      }

    @Override
     public void periodic() {
        //SmartDashboard.putNumber("CANCoder value", corner.getCoder().getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * (180 / Math.PI));
        //SmartDashboard.putNumber()
    }
}