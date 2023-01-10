package frc.lib.team254.geometry;

import frc.lib.team254.util.Util;

import java.text.DecimalFormat;

import static frc.lib.team254.util.Util.kEpsilon;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Rotation2d implements IRotation2d<Rotation2d> {
    protected static final Rotation2d kIdentity = new Rotation2d();

    public static final Rotation2d identity() {
        return kIdentity;
    }

    protected final double cos_angle_;
    protected final double sin_angle_;
    protected double theta_degrees = 0;
    protected double theta_radians = 0;

    public Rotation2d() {
        this(1, 0, false);
    }

    public Rotation2d(double x, double y, boolean normalize) {
        if (normalize) {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            } else {
                sin_angle_ = 0;
                cos_angle_ = 1;
            }
        } else {
            cos_angle_ = x;
            sin_angle_ = y;
        }
	    theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public Rotation2d(final Rotation2d other) {
        cos_angle_ = other.cos_angle_;
        sin_angle_ = other.sin_angle_;
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }
    
    public Rotation2d(double theta_degrees){
    	cos_angle_ = Math.cos(Math.toRadians(theta_degrees));
    	sin_angle_ = Math.sin(Math.toRadians(theta_degrees));
    	this.theta_degrees = theta_degrees;
    }

    public Rotation2d(final Translation2d direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public Rotation2d(final edu.wpi.first.math.geometry.Rotation2d other) {
        cos_angle_ = other.getCos();
        sin_angle_ = other.getSin();
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return new Rotation2d(angle_degrees);
    }

    public double cos() {
        return cos_angle_;
    }

    public double sin() {
        return sin_angle_;
    }

    public double tan() {
        if (Math.abs(cos_angle_) < kEpsilon) {
            if (sin_angle_ >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle_ / cos_angle_;
    }

    public double getRadians() {
        return Math.atan2(sin_angle_, cos_angle_);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    
    public double getUnboundedDegrees(){
    	return theta_degrees;
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     *
     * @param other The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2d rotateBy(final Rotation2d other) {
        return new Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
    }

    public Rotation2d normal() {
        return new Rotation2d(-sin_angle_, cos_angle_, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation2d inverse() {
        return new Rotation2d(cos_angle_, -sin_angle_, false);
    }

    public boolean isParallel(final Rotation2d other) {
        return Util.epsilonEquals(Translation2d.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation2d toTranslation() {
        return new Translation2d(cos_angle_, sin_angle_);
    }
    
    /**
     * @return The pole nearest to this rotation.
     */
    public Rotation2d nearestPole(){
    	double pole_sin = 0.0;
    	double pole_cos = 0.0;
    	if(Math.abs(cos_angle_) > Math.abs(sin_angle_)){
    		pole_cos = Math.signum(cos_angle_);
    		pole_sin = 0.0;
    	}else{
    		pole_cos = 0.0;
    		pole_sin = Math.signum(sin_angle_);
    	}
    	return new Rotation2d(pole_cos, pole_sin, false);
    }

    @Override
    public Rotation2d interpolate(final Rotation2d other, double x) {
        if (x <= 0) {
            return new Rotation2d(this);
        } else if (x >= 1) {
            return new Rotation2d(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(getDegrees());
    }

    @Override
    public double distance(final Rotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Rotation2d)) return false;
        return distance((Rotation2d)other) < Util.kEpsilon;
    }

    @Override
    public Rotation2d getRotation() {
        return this;
    }

    public edu.wpi.first.math.geometry.Rotation2d getWPIRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromDegrees(this.getDegrees());
    }
}
