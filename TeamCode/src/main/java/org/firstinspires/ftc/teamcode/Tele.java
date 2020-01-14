package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@TeleOp
public class Tele extends OpMode{

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
    Servo s;
    boolean dPadDown = false;
    boolean bPressed = false;
    final double SPEED_FACTOR = .30;
    final double ACTUATOR_FACTOR = 1.1;
    StoneDetector detector = new StoneDetector();
    BNO055IMUImpl imu;
    Orientation lastAngles = new Orientation();
    ElapsedTime runtime = new ElapsedTime();
    double runningSum = 0d;
    double lastError = 0d;

    @Override
    public void init() {
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
        s = hardwareMap.servo.get("s");
        rightActuator.setMode(RunMode.STOP_AND_RESET_ENCODER);
        leftActuator.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rightActuator.setMode(RunMode.RUN_WITHOUT_ENCODER);
        leftActuator.setMode(RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");

        imu.initialize(parameters);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public void init_loop(){
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double dt = runtime.seconds();
        runtime.reset();

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x * .4;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        if((rightPlatform.getPosition() == 1 && leftPlatform.getPosition() == 0) || dPadDown){
            frontLeft.setPower(v1 * .15);
            frontRight.setPower(v2 * .15);
            backLeft.setPower(v3 * .15);
            backRight.setPower(v4 * .15);
        } else {
            frontLeft.setPower(v1 * SPEED_FACTOR);
            frontRight.setPower(v2 * SPEED_FACTOR);
            backLeft.setPower(v3 * SPEED_FACTOR);
            backRight.setPower(v4 * SPEED_FACTOR);
        }
        if(dPadDown){
            telemetry.addData("Dpad: ", "Down");
        } else {
            telemetry.addData("Dpad: ", "Up");
        }
        while(gamepad2.b){
            hinge.setPosition(hinge.getPosition() - (.005*.25));
        }
        while(gamepad2.y) {
            hinge.setPosition(hinge.getPosition() + (.005*.25));
            bPressed = true;
        }
        while(gamepad2.a && claw.getPosition() < 1) {
            claw.setPosition(claw.getPosition() + .01);
        } while(gamepad2.x && claw.getPosition() > 0){
            claw.setPosition(claw.getPosition() - .01);
        }

        telemetry.addData("Servo pos", claw.getPosition());

        if(gamepad2.dpad_left){
            s.setPosition(s.getPosition()- .01);
        }
        if(gamepad2.dpad_right){
            s.setPosition(s.getPosition()+ .01);
        }
        if(gamepad1.dpad_down){
            dPadDown = true;
        }
        if(gamepad1.dpad_up){
            dPadDown = false;
        }

        if(gamepad2.right_trigger < 0.05) {
            //if(rightActuator.getCurrentPosition() > 10 && leftActuator.getCurrentPosition() > 10){
            rightActuator.setPower(-gamepad2.left_trigger * ACTUATOR_FACTOR);
            leftActuator.setPower(-gamepad2.left_trigger * ACTUATOR_FACTOR);
            //}
        } else {
            //if(rightActuator.getCurrentPosition() < 3000 && leftActuator.getCurrentPosition() < 3000) {
            rightActuator.setPower(gamepad2.right_trigger * ACTUATOR_FACTOR);
            leftActuator.setPower(gamepad2.right_trigger * ACTUATOR_FACTOR);
            //}
        }

        final double kP = 0.001d;
        final double kI = 0d;
        final double kD = 0d;

        //Let's have the left side follow the right, since the left one lags behind
        //While this gives a slower lifting speed overall, this will be safer as both sides will
        //theoretically always be able to line up with each other during full extension.
        double error = rightActuator.getCurrentPosition() - leftActuator.getCurrentPosition();
        runningSum += error * dt;
        double output = kP * error + kI * runningSum + kD * (error - lastError) / dt;
        lastError = error;
        leftActuator.setPower(leftActuator.getPower() + output);

        if(gamepad1.right_bumper){
            rightPlatform.setPosition(0);
            leftPlatform.setPosition(1);
        } else if(gamepad1.left_bumper){
            rightPlatform.setPosition(1);
            leftPlatform.setPosition(0);
        }
        telemetry.addData("IMU Vel: ", imu.getVelocity());
        telemetry.addData("Stone Detected: ", detector.isDetected());
        telemetry.addLine("Left Position: " + leftActuator.getCurrentPosition());
        telemetry.addLine("Right Position: " + rightActuator.getCurrentPosition());
        telemetry.addLine("Left Power: " + leftActuator.getPower());
        telemetry.addLine("Right Power: " + rightActuator.getPower());
    }
    public void sleep(long millis) {
        try {
            sleep(millis);
        } catch (Exception e) {
            System.out.print(e.getStackTrace());
        }
    }
}
