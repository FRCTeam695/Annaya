package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/*
 * NETWORK TABLES SUBSYSTEM:
 *  - this subsystem controls the output to network tables and shuffleboard (that are not related to the motors)
 */

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
        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();
        xStick = table.getDoubleTopic("XStick").publish();
        xButton = table.getBooleanTopic("xButton").publish();
    }

    public void periodic() {
        xPub.set(x);
        yPub.set(y);
        x += 0.05;
        y += 1.0;
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
