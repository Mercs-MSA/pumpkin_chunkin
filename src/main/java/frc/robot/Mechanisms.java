package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.sim.Pumpkin;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class Mechanisms {
    private final double HEIGHT = 2.0; // Controls the height of the mech2d SmartDashboard
    private final double WIDTH = 4.0; // Controls the height of the mech2d SmartDashboard

    Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
    MechanismLigament2d arm = mech.
                                getRoot("arm", 2.8, 0.15).
                                append(new MechanismLigament2d("arm",  Constants.kArmLength, 180, 6, new Color8Bit(Color.kAliceBlue)));
  
    MechanismLigament2d base = mech.
                                getRoot("base", 2.5, 0.1).
                                append(new MechanismLigament2d("base", 0.75, 0, 6, new Color8Bit(Color.kCyan)));
  
    MechanismLigament2d vertical_stop = mech.
                                getRoot("vertical_stop", 2.75, 0.1).
                                append(new MechanismLigament2d("vertical_stop", 0.5, 90, 6, new Color8Bit(Color.kCyan)));

    MechanismLigament2d pumpkin = mech.
                                getRoot("pumpkin", 3.3, 0.18).
                                append(new MechanismLigament2d("pumpkin", 0.05, 0, 6, new Color8Bit(Color.kOrange)));

    public final Pumpkin m_pumpkin = new Pumpkin(3.3,0.18,0,0);
    /**
     * Runs the mech2d widget in GUI.
     *  
     * This utilizes GUI to simulate and display a TalonFX and exists to allow users to test and understand 
     * features of our products in simulation using our examples out of the box. Users may modify to have a 
     * display interface that they find more intuitive or visually appealing.
     */                            
    public void update(double position, boolean cap_or_free) {
        arm.setAngle(position); // expects degrees
        if (cap_or_free) {
            m_pumpkin.update_captive_kinematics(2.7 + Math.cos(Units.degreesToRadians(position))*(Constants.kArmLength - pumpkin.getLength()), 0.18 + Math.sin(Units.degreesToRadians(position))*(Constants.kArmLength - pumpkin.getLength()), 0.020);
        }
        else{
            m_pumpkin.update_free_kinematics(0.020);
        }

        mech.getRoot("pumpkin", 0, 0).setPosition(m_pumpkin.get_position()[0], m_pumpkin.get_position()[1]);
        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }                             
}