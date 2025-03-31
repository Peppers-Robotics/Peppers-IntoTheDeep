package com.ftc.bumblebee.Followers;

import com.ftc.bumblebee.Localizers.Localizer;
import com.ftc.bumblebee.Localizers.Pose2d;
import com.ftc.bumblebee.Math.PIDCoefficients;
import com.ftc.bumblebee.Math.PIDController;

public class HolonomicFollower {
    private PIDController translational, heading;
    public PIDCoefficients translationalCoefficients, headingCoefficients;
    private Localizer localizer;
    private Pose2d targetPosition;
    public HolonomicFollower(PIDCoefficients translationalCoeff, PIDCoefficients headingCoeff, Localizer localizer){
        this.translationalCoefficients = translationalCoeff;
        this.headingCoefficients = headingCoeff;
        this.localizer = localizer;
    }

    public void setTargetPosition(Pose2d target){
        this.targetPosition = target;
    }

    public Pose2d getDriveCoefficients(){
        translational.setCoefficients(translationalCoefficients.p, translationalCoefficients.i, translationalCoefficients.d);
        heading.setCoefficients(headingCoefficients.p, headingCoefficients.i, headingCoefficients.d);
        double translationalError = Math.hypot(targetPosition.x - localizer.getCurrentPosition().x, targetPosition.y - localizer.getCurrentPosition().x);
        double transaltionalVelo = Math.hypot(localizer.getCurrentelocity().x, localizer.getCurrentelocity().y);
        double headingError = targetPosition.h - localizer.getCurrentPosition().h;
        headingError = Pose2d.NormalizeRadians(headingError);

        double Tpow = translational.output(translationalError, transaltionalVelo);
        double Hpow = heading.output(headingError, localizer.getCurrentelocity().h);

        Pose2d normal = new Pose2d(localizer.getCurrentPosition().x - targetPosition.y, localizer.getCurrentPosition().y - targetPosition.y, headingError);
        double alpha = Math.atan2(normal.y, normal.x);

        double X = Tpow * Math.cos(alpha);
        double Y = Tpow * Math.sin(alpha);
        return new Pose2d(X, Y, Hpow);
    }
}
