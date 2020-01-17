package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class FoundationAuton extends OpMode {
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor leftActuator;
    DcMotor rightActuator;
    Servo claw;
    Servo hinge;
    Servo leftPlatform;
    Servo rightPlatform;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .3, correction;
    int step = 0;

    @Override
    public void init(){
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        leftActuator = hardwareMap.dcMotor.get("leftActuator");
        rightActuator = hardwareMap.dcMotor.get("rightActuator");
        claw = hardwareMap.servo.get("claw");
        hinge = hardwareMap.servo.get("hinge");
        leftPlatform = hardwareMap.servo.get("leftPlatform");
        rightPlatform = hardwareMap.servo.get("rightPlatform");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }
    @Override
    public void init_loop(){
        leftActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void stopAllMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    @Override
    public void loop(){
        if(step == 0){
            hinge.setPosition(.8);
            moveForward(1200);
        } else if(step == 1){
            strafeRight(800);
        } else if(step == 2){
            rotateRight(160);
        } else if(step == 3){
            moveBackward(500);
        } else if(step == 4){
            rightPlatform.setPosition(1);
            leftPlatform.setPosition(0);
            sleep(300);
        } else if(step == 5){
            moveForward(2300);
        } else if(step == 6){
            moveBackward(200);
        } else if(step == 7){
            rightPlatform.setPosition(0);
            leftPlatform.setPosition(1);
        } else if(step == 8){
            moveForward(200);
        } else if(step == 9){
            strafeRight(4000);
        } else if(step == 10){
          rotateLeft(180 - Math.abs(getAngle()));
        } else if(step == 11){
            moveBackward(1500);
        } else if(step == 12){
            strafeLeft(4000);
        } else if(step == 13){
            moveForward(2700);
        } else if(step == 14){
            hinge.setPosition(.3);
        } else if(step == 15){
            strafeRightFast(1800);
        }
        stopAllMotors();
        step++;
        sleep(300);

    }

    public void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            System.out.print(e.getStackTrace());
        }
    }
    public void moveForward(long millis){
        frontLeft.setPower(-.3);
        frontRight.setPower(-.3);
        backLeft.setPower(-.3);
        backRight.setPower(-.3);
        sleep(millis);
    }


    public void moveBackward(long millis){
        frontLeft.setPower(.3);
        frontRight.setPower(.3);
        backLeft.setPower(.3);
        backRight.setPower(.3);
        sleep(millis);
    }

    public void strafeLeft(long millis){
        frontLeft.setPower(.3);
        backLeft.setPower(-.3);
        frontRight.setPower(-.3);
        backRight.setPower(.3);
        sleep(millis);
    }

    public void strafeRight(long millis){
        frontLeft.setPower(-.3);
        backLeft.setPower(.3);
        frontRight.setPower(.3);
        backRight.setPower(-.3);
        sleep(millis);
    }

    public void strafeRightFast(long millis){
        frontLeft.setPower(-.6);
        backLeft.setPower(.6);
        frontRight.setPower(.6);
        backRight.setPower(-.6);
        sleep(millis);
    }

    public void rotateRight(double degrees){
        resetAngle();
        backRight.setPower(.3);
        backLeft.setPower(-.3);
        frontRight.setPower(.3);
        frontLeft.setPower(-.3);
        while(Math.abs(getAngle()) < degrees){
            telemetry.addData("Current Angle: ", getAngle());
            telemetry.update();
        }
        stopAllMotors();
    }

    public void turnToDegree(int degrees){
        if(getAngle() > degrees){
            rotateLeft(getAngle() - degrees);
        } else if(getAngle() < degrees){
            rotateRight(degrees - getAngle());
        }
    }

    public void lowerActuator(){
        leftActuator.setTargetPosition(-900);
        rightActuator.setTargetPosition(-900);
        leftActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftActuator.setPower(-.3);
        rightActuator.setPower(-.3);
        while(leftActuator.isBusy() && rightActuator.isBusy()){

        }
        leftActuator.setPower(0);
        rightActuator.setPower(0);
    }

    public void rotateLeft(double degrees){
        resetAngle();
        backRight.setPower(-.3);
        backLeft.setPower(.3);
        frontRight.setPower(-.3);
        frontLeft.setPower(.3);
        while(Math.abs(getAngle()) < degrees){
            telemetry.addData("Current Angle: ", getAngle());
            telemetry.update();
        }
        stopAllMotors();
    }

    public void driveStraight(){

    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }



}
