package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


//This is the main subsystem for network tables

public class NetworkTablesSubsystem extends SubsystemBase {
    DoublePublisher xPub;
    DoublePublisher yPub;
  
    BooleanPublisher xButton;
    DoublePublisher xStick;

    private double x = 0;
    private double y = 0;

    public int maxCnt = 50;
    public int count = 0;

    public NetworkTablesSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");


    }

    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean NTCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}
