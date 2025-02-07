package org.firstinspires.ftc.robotcontroller;

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class AutoDavid extends LinearOpMode {

    //    Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MID = null;
    private DcMotor MIA = null;
    private DcMotor MDD = null;
    private DcMotor MDA = null;
    private ColorSensor sensorColor;
    //private Servo myServo=null;
    boolean hasExecuted = false;
    float[] hsv = {0.0f, 0.0f, 0.0f};
    float rojo = 0, verde = 0, azul = 0;

    private Servo SIM = null;
    private Servo SDM = null;
    private Servo SGO = null;

    float SCALE_FACTOR = 255;
    static final double TICKS_PER_REV = 537.6;
    static final double CIRCUNFERENCIA_CM = 14;// Circunferencia de la rueda en cm
    static final double TICKS_PER_CM = TICKS_PER_REV / CIRCUNFERENCIA_CM;

    // Control/Expansion Hub IMU giroscopio
    private IMU             imu         = null;
    @Override
    public void runOpMode() {

        sensorColor = hardwareMap.colorSensor.get("sensorColor");
        // Inicializar los motores
        SDM = hardwareMap.get(Servo.class, "SDM");
        SIM = hardwareMap.get(Servo.class, "SIM");
        SGO = hardwareMap.get(Servo.class, "SGO");
        MID = hardwareMap.get(DcMotor.class, "MID");
        MIA = hardwareMap.get(DcMotor.class, "MIA");
        MDD = hardwareMap.get(DcMotor.class, "MDD");
        MDA = hardwareMap.get(DcMotor.class, "MDA");

        MID.setDirection(DcMotor.Direction.FORWARD);//otra opcon es MID.setDirection(DcMotor.Direction.REVERSE);
        MIA.setDirection(DcMotor.Direction.FORWARD);
        MDD.setDirection(DcMotor.Direction.FORWARD);
        MDA.setDirection(DcMotor.Direction.FORWARD);

    //giroscopio inicio
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
   //giroscopio fin

        // Resetear los encoders
        MID.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MIA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// girosciop cambia 1 linea
        imu.resetYaw();



// girosciop cambia inicia
        // waitForStart();
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }
// girosciop cambia fin

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int targetTicks = (int) (70 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);


            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(1);
            MDD.setPower(-1);
            MDA.setPower(-1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 2paso
            targetTicks = (int) (30 * TICKS_PER_CM);
            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(-1);
            MIA.setPower(-1);
            MDD.setPower(1);
            MDA.setPower(1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Path", "Complete");
            telemetry.update();
            // 3paso
            targetTicks = (int) (70 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(-1);
            MDD.setPower(1);
            MDA.setPower(-1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Path", "Complete");
            telemetry.update();

            //paso4

            targetTicks = (int) (65 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(1);
            MDD.setPower(-1);
            MDA.setPower(-1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //paso5

            targetTicks = (int) (70 * TICKS_PER_CM);
            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(-1);
            MDD.setPower(1);
            MDA.setPower(-1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Path", "Complete");
            telemetry.update();


            //paso6

            targetTicks = (int) (62 * TICKS_PER_CM);
            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(1);
            MDD.setPower(1);
            MDA.setPower(1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Path", "Complete");
            telemetry.update();



            //paso 7
            targetTicks = (int) (130 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(-1);
            MIA.setPower(-1);
            MDD.setPower(1);
            MDA.setPower(1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            telemetry.addData("Path", "Complete");
            telemetry.update();

            //paso8
            targetTicks = (int) (140 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(1);
            MIA.setPower(1);
            MDD.setPower(-1);
            MDA.setPower(-1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //paso 9
            targetTicks = (int) (20 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(-1);
            MIA.setPower(1);
            MDD.setPower(-1);
            MDA.setPower(1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("Path", "Complete");
            telemetry.update();
            //paso 10
            targetTicks = (int) (140 * TICKS_PER_CM);

            // Configurar el objetivo de los encoders
            MID.setTargetPosition(targetTicks);
            MIA.setTargetPosition(targetTicks);
            MDD.setTargetPosition(targetTicks);
            MDA.setTargetPosition(targetTicks);

            // Cambiar el modo de los motores para correr hacia la posición
            MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Establecer la potencia del motor para avanzar
            MID.setPower(-1);
            MIA.setPower(-1);
            MDD.setPower(1);
            MDA.setPower(1);

            // Esperar hasta que los motores lleguen a la posición
            while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                telemetry.addData("Path", "Driving 50cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                telemetry.update();
            }

            // Detener todos los motores
            MID.setPower(0);
            MIA.setPower(0);
            MDD.setPower(0);
            MDA.setPower(0);

            // Cambiar el modo de los motores para usar el encoder
            MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            telemetry.addData("Path", "Complete");
            telemetry.update();


            //recoger
            SGO.setPosition(-0.30);
            sensorColor.enableLed(true);


            rojo = sensorColor.red();
            azul = sensorColor.blue();
            verde = sensorColor.green();

            Color.RGBToHSV((int) (rojo * SCALE_FACTOR), (int) (verde * SCALE_FACTOR), (int) (azul * SCALE_FACTOR), hsv);
            SDM.setPosition(0.285);
            SIM.setPosition(0.40);

            if (!hasExecuted && azul > 0.5) {
                SGO.setPosition(.36);
                hasExecuted = true;
                sensorColor.enableLed(false);

            }else{
                //a la izquierda
                targetTicks = (int) (15* TICKS_PER_CM);

                // Configurar el objetivo de los encoders
                MID.setTargetPosition(targetTicks);
                MIA.setTargetPosition(targetTicks);
                MDD.setTargetPosition(targetTicks);
                MDA.setTargetPosition(targetTicks);

                // Cambiar el modo de los motores para correr hacia la posición
                MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Establecer la potencia del motor para avanzar
                MID.setPower(1);
                MIA.setPower(-1);
                MDD.setPower(1);
                MDA.setPower(-1);

                // Esperar hasta que los motores lleguen a la posición
                while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                    telemetry.addData("Path", "Driving cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                    telemetry.update();
                }

                // Detener todos los motores
                MID.setPower(0);
                MIA.setPower(0);
                MDD.setPower(0);
                MDA.setPower(0);

                // Cambiar el modo de los motores para usar el encoder
                MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //a la derecha
                targetTicks = (int) (30* TICKS_PER_CM);

                // Configurar el objetivo de los encoders
                MID.setTargetPosition(targetTicks);
                MIA.setTargetPosition(targetTicks);
                MDD.setTargetPosition(targetTicks);
                MDA.setTargetPosition(targetTicks);

                // Cambiar el modo de los motores para correr hacia la posición
                MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Establecer la potencia del motor para avanzar
                MID.setPower(-1);
                MIA.setPower(1);
                MDD.setPower(-1);
                MDA.setPower(1);

                // Esperar hasta que los motores lleguen a la posición
                while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
                    telemetry.addData("Path", "Driving cm to %7d :%7d", targetTicks, MID.getCurrentPosition());
                    telemetry.update();
                }

                // Detener todos los motores
                MID.setPower(0);
                MIA.setPower(0);
                MDD.setPower(0);
                MDA.setPower(0);

                // Cambiar el modo de los motores para usar el encoder
                MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



                telemetry.addData("Path", "Complete");
                telemetry.update();

                sensorColor.enableLed(true);
            }






        }
    }
}