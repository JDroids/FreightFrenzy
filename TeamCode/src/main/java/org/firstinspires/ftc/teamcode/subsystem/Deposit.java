package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Deposit extends SubsystemBase {
    private final HardwareMap hardwareMap;
    private DcMotorEx extensionMotor;
    private Servo flipServo;
    private Servo blockerServo;

    private double targetHeight;

    public static double offset = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double extensionMotorPower = 1.0;

    public static double RETRACTED_HEIGHT = 0.5;
    public static double LEVEL_1_HEIGHT = 7;
    public static double LEVEL_2_HEIGHT = 14;
    public static double LEVEL_3_HEIGHT = 30;

    public static double RETRACTED_POSITION = 0.3;
    public static double MOVING_POSITION = 0.5;
    public static double DEPLOY_POSITION = 1.0;
    public static double TELEOP_DEPLOY_POSITION = 0.8;

    public static double BLOCKER_SERVO_OPEN = 0.5;
    public static double BLOCKER_SERVO_BLOCKING = 0.24;

    public boolean disableBlocker;

    private enum State {
        RETRACTED,
        RETRACTING,
        WAITING_ON_BLOCKER_SERVO,
        GOING_TO_HEIGHT,
        DEPLOY,
        TELEOP_DEPLOY
    }

    private State state = State.RETRACTED;

    public Deposit(HardwareMap hardwareMap, boolean isAuto) {

        this.hardwareMap = hardwareMap;

        extensionMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        flipServo = hardwareMap.get(Servo.class, "depositServo");
        blockerServo = hardwareMap.get(Servo.class, "blockerServo");

        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isAuto) {
            extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            state = State.GOING_TO_HEIGHT;
        }

        disableBlocker = !isAuto;

        extensionMotor.setTargetPosition(inchesToTicks(RETRACTED_HEIGHT));
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flipServo.setPosition(MOVING_POSITION);
    }

    @Override
    public void periodic() {
        extensionMotor.setTargetPosition(inchesToTicks(targetHeight + offset));
        extensionMotor.setPower(extensionMotorPower);

        switch (state) {
            case RETRACTED:
                flipServo.setPosition(RETRACTED_POSITION);

                if (timer.seconds() > 0.5) {
                    blockerServo.setPosition(
                            disableBlocker ? BLOCKER_SERVO_OPEN : BLOCKER_SERVO_BLOCKING);
                }
                break;
            case WAITING_ON_BLOCKER_SERVO:
                flipServo.setPosition(RETRACTED_POSITION);
                blockerServo.setPosition(BLOCKER_SERVO_OPEN);

                if (timer.seconds() > 0.2 || disableBlocker) {
                    extensionMotorPower = 1.0;
                    state = State.GOING_TO_HEIGHT;
                }

                break;
            case RETRACTING:
                flipServo.setPosition(MOVING_POSITION);
                blockerServo.setPosition(BLOCKER_SERVO_OPEN);

                if (Math.abs(
                        extensionMotor.getCurrentPosition() - extensionMotor.getTargetPosition())
                        < inchesToTicks(2.0)) {
                    timer.reset();
                    state = State.RETRACTED;
                }
                break;
            case GOING_TO_HEIGHT:
                flipServo.setPosition(MOVING_POSITION);
                blockerServo.setPosition(BLOCKER_SERVO_OPEN);
                break;
            case DEPLOY:
                flipServo.setPosition(DEPLOY_POSITION);
                blockerServo.setPosition(BLOCKER_SERVO_OPEN);

                if (timer.seconds() > 0.5) {
                    targetHeight = RETRACTED_HEIGHT;
                    state = State.RETRACTING;
                }
                break;
            case TELEOP_DEPLOY:
                flipServo.setPosition(TELEOP_DEPLOY_POSITION);
                blockerServo.setPosition(BLOCKER_SERVO_OPEN);

                if (timer.seconds() > 0.5) {
                    targetHeight = RETRACTED_HEIGHT;
                    state = State.RETRACTING;
                }
                break;
        }
    }

    public void goToHeight(double targetHeight) {
        this.targetHeight = targetHeight;

        if (state == State.RETRACTED) {
            extensionMotorPower = 0;
            timer.reset();
            state = State.WAITING_ON_BLOCKER_SERVO;
        }
        else {
            extensionMotorPower = 1;
            state = State.GOING_TO_HEIGHT;
        }
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public void goToLevel1() {
        goToHeight(LEVEL_1_HEIGHT);
    }

    public void goToLevel2() {
        goToHeight(LEVEL_2_HEIGHT);
    }

    public void goToLevel3() {
        goToHeight(LEVEL_3_HEIGHT);
    }

    public void retract() {
        targetHeight = RETRACTED_HEIGHT;
        state = State.RETRACTING;
    }

    public void deploy() {
        if (state == State.RETRACTING || state == State.RETRACTED) {
            return;
        }

        timer.reset();
        state = State.DEPLOY;
    }

    public void teleopDeploy() {
        if (state == State.RETRACTING || state == State.RETRACTED) {
            return;
        }

        timer.reset();
        state = State.TELEOP_DEPLOY;
    }

    public boolean isBusy() {
        return extensionMotor.isBusy()
                || state == State.DEPLOY
                || state == State.WAITING_ON_BLOCKER_SERVO;
    }

    private static int inchesToTicks(double inches) {
        final double DIAMETER = 48 / 25.4;
        final double CIRCUMFERENCE = DIAMETER * Math.PI;

        final double TICKS_PER_ROTATION = 28 * 19.2;

        return (int) (inches / CIRCUMFERENCE * TICKS_PER_ROTATION);
    }
}
