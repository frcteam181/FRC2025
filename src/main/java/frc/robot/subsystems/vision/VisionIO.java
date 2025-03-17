package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean ntConnected = false;
  }

  @AutoLog
  class AprilTagVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  @AutoLog
  class ObjDetectVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  default void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {}

  default void setRecording(boolean active) {}
}