package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {

  // Elevator
  public static class ElevatorConstants {

    public static final int LEADER_ID = 14;
    public static final int FOLLOWER_ID = 5;
    public static final int CANCODER_ID = 31;
    public static final String CANBUS = "rio";

    public static final double maxTravel = 1.45; // meters (COMP 1.6)
  }

  // Extender
  public static class ExtenderConstants {

    public static final int PIVOT_ID = 4;
    public static final int CANCODER_ID = 29;
    public static final int GRIPPER_ID = 16;
    public static final String CANBUS = "rio";
  }

  // Beak
  public static class BeakConstants {

    public static final int PIVOT_ID = 3;
    public static final int ROLLER_ID = 15;
    public static final int CANCODER_ID = 30;
    public static final String CANBUS = "rio";
  }
}
