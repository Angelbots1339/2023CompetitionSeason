// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class Mech2dManger {
    private static Mech2dManger instance = null;
    
    private Mechanism2d mech2d = new Mechanism2d(3,3);

    private MechanismRoot2d root = mech2d.getRoot("SuperStructure", 2, 0);

    private MechanismLigament2d elevator;

    private MechanismLigament2d wrist;

    public static Mech2dManger getInstance() {
        if (instance == null) {
            instance = new Mech2dManger();
        }
        return instance;
    }

    private Mech2dManger() {
        MechanismLigament2d base = root.append(new MechanismLigament2d("Base", 0.197231, 90));
        elevator = base.append(new MechanismLigament2d("Elevator", 1, 30));
        MechanismLigament2d carriage = elevator.append(new MechanismLigament2d("Carriage", 0.4826, 90));
        wrist = carriage.append(new MechanismLigament2d("wrist", 0.2, -50, 20, new Color8Bit(Color.kBlue)));
       // SmartDashboard.putData("elevator", mech2d);
    }

    public MechanismLigament2d getWrist() {
        return wrist;
    }

    public MechanismLigament2d getElevator() {
        return elevator;
    }
}
