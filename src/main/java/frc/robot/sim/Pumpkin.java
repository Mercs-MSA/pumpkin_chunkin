package frc.robot.sim;

import frc.robot.Constants;

public class Pumpkin{

    public double prior_position_x, prior_position_y, position_x, position_y, velocity_x, velocity_y;

    public Pumpkin(double pos_x, double pos_y, double vel_x, double vel_y) {
        prior_position_x = pos_x;
        prior_position_y = pos_y;
        position_x = pos_x;
        position_y = pos_y;
        velocity_x = vel_x;
        velocity_y = vel_y;      
    }

    public void update_captive_kinematics(double new_position_x, double new_position_y, double time_interval) {
        prior_position_x = position_x;
        prior_position_y = position_y;
        position_x = new_position_x;
        position_y = new_position_y;
        velocity_x = (position_x - prior_position_x) / time_interval;
        velocity_y = (position_y - prior_position_y) / time_interval;
    }

    public void update_free_kinematics(double time_interval) {
        // velocity_x assumed unchanged
        velocity_y = velocity_y + Constants.gravity_acceleration * time_interval;
        position_x = position_x + (velocity_x * time_interval);
        position_y = position_y + (velocity_y * time_interval) + (0.5 * Constants.gravity_acceleration * time_interval * time_interval);
    }

    public double[] get_position() {
        double pos[] = {position_x, position_y};
        return pos;
    }

    public double[] get_velocity() {
        double vel[] = {velocity_x, velocity_y};
        return vel;
    }
}