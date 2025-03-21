package com.ftc.bumblebee.Localizers;


public abstract class Localizer {

    /**
    * Returns the current <b>estimated</b> position
    * */
    public abstract Pose2d getCurrentPosition();

    /**
     * Returns the current <b>estimated</b> velocity
     * */
    public abstract Pose2d getCurrentelocity();

    /**
     *  Updates the current position based on a beacon, a precise measurement of the current position
     *  based on external sensors like apriltags or heading received from external imu measurements
    * */
}
