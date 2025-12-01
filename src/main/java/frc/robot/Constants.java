// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // make april tag layout
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // motor IDs
  public static class CanIDs {

    public static final int LEFT_ELEVATOR_TALON = 10;
    public static final int RIGHT_ELEVATOR_TALON = 11;

    public static final int INTAKE_TALON = 12;
    public static final int INTAKE_TALON2 = 13;
    public static final int ALGAE_TALON = 14;

    public static final int LEFT_PIVOT_TALON = 16;
    public static final int RIGHT_PIVOT_TALON = 17;

    public static final int ELEVATOR_CANDI = 20;
    public static final int PIVOT_CANCODER = 21;

    public static final int INTAKE_CANRANGE = 23;
  }

  // intake constants
  public static class IntakeConstants {

    // speeds and stuff
    public static final double CORAL_INTAKE_SPEED = 0.4 * 1.2 * 1.1;
    public static final double CORAL_OUTPUT_SPEED = 0.6;
    public static final double L1_OUTPUT_SPEED = 0.4;

    public static final double ALGAE_HOLD = -0.125;
    public static final double ALGAE_INTAKE_SPEED = -0.25;
    public static final double ALGAE_OUTPUT_SPEED = 1;

    public static final double SUPPLY_CURRENT = 35;

    // pid + gaer stuff
    public static final double GEAR_RATIO = 4;
    public static final Slot0Configs GAINS =
        new Slot0Configs().withKP(1).withKI(0).withKD(0).withKA(0).withKS(0).withKV(0);
  }

  // pivot constatnts
  public static class PivotConstants {

    // gains +gear ratio
    public static final double GEAR_RATIO = 961 / 36;

    public static final Slot0Configs GAINS =
        new Slot0Configs()
            .withKP(30)
            .withKI(0)
            .withKD(0.3)
            .withKA(0)
            .withKS(0.19)
            .withKV(0)
            .withKG(0.27)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final double MM_ACCELERATION = 10;
    public static final double MM_CRUISE_VELOCITY = 10;

    // postiiotns
    public static final double START_ANGLE = -0.024902;
    public static final double STOW_ANGLE = -0.5;
    public static final double GROUND_INTAKE_ANGLE = -10.591797;
    public static final double ALGAE_INTAKE_ANGLE = -10.591797;
    public static final double SCORING_ANGLE = -0.5;
    public static final double PROCESSOR_SCORING_ANGLE = -8;

    // other
    public static final double CANCODER_DISCONTINUITY = 0.5;
    public static final double CANCODER_OFFSET = -0.434326171875;

    public static final double SETPOINT_TOLERANCE = 0.03125;
  }

  public static class ElevatorConstants {

    // positions
    public static final double L1_SCORE_POSITION = -0.03;
    public static final double L2_SCORE_POSITION = -2.538818;
    public static final double L3_SCORE_POSITION = -2.0; // L3 not reachable on robot
    public static final double L2_ALGAE_POSITION = -0.03;
    public static final double L3_ALGAE_POSITION = -2.6;
    public static final double PROCESSOR_SCORE_POSITION = -1;
    public static final double CAM_POSITION = -1;
    public static final double ELEVATOR_MAX = -2.6;

    public static final double DEFAULT_POSITION = -0.03;

    // MM constants and gains+gear ration

    public static final double MM_CRUISE_VELOCITY = 100;

    static String pluh = "hihi";
    static int len = pluh.length();
    public static final double MM_JERK = len * 1000000;
    public static final double MM_UPWARDS_ACCELERATION = 50;
    public static final double MM_BRALE_ACCELERATION = 25;
    public static final double MM_DOWNWARD_ACCELERATION = 50;

    public static final double GEAR_RATIO = 12 / 1;

    public static final Slot0Configs GAINS =
        new Slot0Configs()
            .withKP(1)
            .withKI(0.1)
            .withKD(0.0)
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKG(0.36)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final double SETPOINT_TOLERANCE = 0.1;
  }
}
