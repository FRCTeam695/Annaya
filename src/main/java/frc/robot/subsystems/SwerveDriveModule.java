package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
 * SWERVE DRIVE MODULE:
 *   - this object class will define a single corner of a swerve drive drivetrain with TalonFX motors.
 */

public class SwerveDriveModule {

     //VARIABLES BELOW

    private final TalonFX driveMotor;
    private final CANcoder encoder;
    private final TalonFX angleMotor;

    private final PIDController pid;

    public SwerveDriveModule(int driveID, int angleID, int encoID) {

        driveMotor = new TalonFX(driveID);
        angleMotor = new TalonFX(angleID);
        encoder = new CANcoder(encoID);
        pid = new PIDController(0.1, 0.000001, 0.00001);

    }

    /**
     * Configures the CANcoder,  this involves giving it an offset
     * 
     * @param offset the offset to give the CANcoder
     */
    public void configCANcoder(double offset){
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = offset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(cancoderConfigs);
    }

    protected double getCANCoderRadians(){
        double angleRad = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        //SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Radians", angleRad);
        //SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Degrees", Math.toDegrees(angleRad));
        return angleRad;
    }


     public void setSpeedAndAngle(double FWD, double STR, double RCW, double trackWidth, double trackLength) {

        //max RPM is 5700
        //computing the R value using pythagorean theorem with trackWidth and trackLength
        double hypotenuse = Math.sqrt(Math.pow(0.5 * trackWidth, 2) + Math.pow(0.5 * trackLength, 2));
        
        //defining the effect of RCW on FWD as well as the effect of RCW on STR
        double RCW_FWD = RCW * (0.5 * trackLength / hypotenuse);
        double RCW_STR = RCW * (0.5 * trackWidth / hypotenuse);
        
        //thus, for CORNER 1, the below are new FWD and STR values for given inputs as based on RCW's impact on both
        //these calcs only work for CORNER 1 (only one corner)
        double new_FWD = FWD - RCW_FWD;
        double new_STR = STR + RCW_STR;

        //below is the calculation of both steering angle as well as speed for this swerve module based on the new FWD and STR values and the RCW value
        double speed = Math.sqrt(Math.pow(new_FWD, 2) + Math.pow(new_STR, 2));
        double steeringAngle = Math.toDegrees(Math.atan(new_STR / new_FWD));

        
        //NOTE: the code lines to send angles and speed to appropriate motors are commented right now as i am working on some corrections
        //however, they are the lines that i will uncomment when ready to test! 

        //applying the speed and steering angle setpoints to PID controllers for the two motors
        //driveMotor.set(speed);
        //angleMotor.set(pid.calculate(getCANCoderRadians() * (180 / Math.PI), steeringAngle));

        SmartDashboard.putNumber("Drive motor speed: ", speed);

        System.out.println(getCANCoderRadians() * (180 / Math.PI));

        SmartDashboard.putNumber("Encoder absolute position", encoder.getAbsolutePosition().getValueAsDouble()* 2 * Math.PI * (180 / Math.PI));
        SmartDashboard.putNumber("Steering angle: ", steeringAngle);
        SmartDashboard.putNumber("Angle motor angle: ", getCANCoderRadians() * (180 / Math.PI));
     }

     //just for testing
     public void spin(double amount) {
        driveMotor.set(amount);
        System.out.println("The motor is spinning");
     }

     public CANcoder getCoder() {
        return encoder;
     }
}