package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  byte idMotorIzqAdel = 3;
  byte idMotorIzqAtras = 4;

  byte idMotorDerAdel = 1;
  byte idMotorDerAtras = 2;

  byte idMotorInternoShooterIzq = 5;
  byte idMotorShooterInternoDer = 6;

  byte idCajaShooterAtras = 7;
  byte idCajaShooterAdelante = 8;
  
  byte idMotorExternoShooterDer = 9;
  byte idMotorExternoShooterIzq = 10;

  byte idMotorBrazo = 11;
  byte idMotorMano = 12;


  CANSparkMax motorIzqAdelante = new CANSparkMax(idMotorIzqAdel, MotorType.kBrushless);
  CANSparkMax motorIzqAtras = new CANSparkMax(idMotorIzqAtras, MotorType.kBrushless);
  CANSparkMax motorDerAdelante = new CANSparkMax(idMotorDerAdel, MotorType.kBrushless);
  CANSparkMax motorDerAtras = new CANSparkMax(idMotorDerAtras, MotorType.kBrushless);

  SparkMaxPIDController pidDer = motorDerAdelante.getPIDController();
  SparkMaxPIDController pidIzq = motorIzqAdelante.getPIDController();

  MotorControllerGroup Izq = new MotorControllerGroup(motorIzqAdelante, motorIzqAtras);

  RelativeEncoder encoderIzqAdel;
  RelativeEncoder encoderIzqAtras;
  RelativeEncoder encoderDerAdel;
  RelativeEncoder encoderDerAtras;
    
  MotorControllerGroup motoresIzq  = new MotorControllerGroup(motorIzqAdelante, motorIzqAtras);
  MotorControllerGroup motoresDer = new MotorControllerGroup(motorDerAdelante, motorDerAtras);

  DifferentialDrive chasis = new DifferentialDrive(motoresIzq, motoresDer);

  double valorEncoderDerAdel;
  double valorEncoderIzqAdel;
  
  CANSparkMax motorInternoShooterIzq = new CANSparkMax(idMotorInternoShooterIzq, MotorType.kBrushless);
  CANSparkMax motorInternoShooterDer = new CANSparkMax(idMotorShooterInternoDer, MotorType.kBrushless);
  WPI_TalonSRX motorExternoShooterDer = new WPI_TalonSRX(idMotorExternoShooterDer);
  WPI_TalonSRX motorExternoShooterIzq = new WPI_TalonSRX(idMotorExternoShooterIzq);
  MotorControllerGroup motoresExternosShooter = new MotorControllerGroup(motorExternoShooterDer, motorExternoShooterIzq);
  
  
  CANSparkMax motorCajaShooterAdelante = new CANSparkMax(idCajaShooterAdelante, MotorType.kBrushless);
  CANSparkMax motorCajaShooterAtras = new CANSparkMax(idCajaShooterAtras, MotorType.kBrushless);
  MotorControllerGroup motoresCajaShooter = new MotorControllerGroup(motorCajaShooterAdelante, motorCajaShooterAtras);
  
  RelativeEncoder encoderCajaShootAdel;
  RelativeEncoder encoderCajaShootAtras;
  
  double valorEncoderCajaAdel;
  double valorEncoderCajaAtras;

  
  WPI_VictorSPX motorBrazo = new WPI_VictorSPX(idMotorBrazo);
  WPI_VictorSPX motorMano = new WPI_VictorSPX(idMotorMano);

  Solenoid velocidades = new Solenoid(PneumaticsModuleType.REVPH, 1);
    


  double relacionBaja = 1.4; 
  double relacionAlta = 0.9;
  double relacionUsada =  relacionBaja;

  double diametroLlanta = 0.1524; // Metros
  double distanciaRecorrida;  

  double velocidadShooter = 0;

  
  AHRS NavX = new AHRS(Port.kMXP);
  
  //
  double velocidadPositivaBaja = 0.25D;
  double velocidadNegativaBaja = -0.25D;
  double velocidadPositivaMedia = 0.5D;
  double velocidadNegativaMedia = -0.5D;
  
  boolean tank = true;
  boolean tiempo_activo = true;
  boolean velocidad;
  boolean movimiento1 = false;  
  boolean movimiento2 = false;  
  boolean movimiento3 = false;  


  Timer tiempo = new Timer();

  
  Joystick controlDriver = new Joystick(0);
  Joystick controlPlacer = new Joystick(1);

  /*----###################################################################################################3-- */
  @Override
  public void robotInit() {
    //SetUp Encoders
    encoderIzqAdel = motorIzqAdelante.getEncoder();
    encoderIzqAtras = motorIzqAtras.getEncoder();
    encoderDerAdel = motorDerAdelante.getEncoder();
    encoderDerAtras = motorDerAtras.getEncoder();

    //SetUp spark Max Chasis
    //roult tiene chasis en coast
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);

    //SetUp inverted Motors
    motoresDer.setInverted(false);
    motoresIzq.setInverted(!motoresDer.getInverted());

    motorDerAtras.follow(motorDerAdelante);
    motorIzqAtras.follow(motorIzqAdelante);
    //Sensor Reset 
    resetEncodersChasis();
    
    //
    encoderCajaShootAdel = motorCajaShooterAdelante.getEncoder();
    encoderCajaShootAtras = motorCajaShooterAtras.getEncoder();

    motorCajaShooterAdelante.setIdleMode(IdleMode.kBrake);
    motorCajaShooterAdelante.setIdleMode(IdleMode.kBrake);

    motorInternoShooterDer.setIdleMode(IdleMode.kBrake);
    motorInternoShooterIzq.setIdleMode(IdleMode.kBrake);


    motorExternoShooterIzq.setInverted(false);
    motorExternoShooterDer.setInverted(!motorExternoShooterIzq.getInverted());

    CameraServer.startAutomaticCapture( 0);
    
  }

  @Override
  public void robotPeriodic() {

    /*valorEncoderIzqAdel = encoderIzqAdel.getPosition();
    valorEncoderDerAdel = -encoderDerAdel.getPosition();

    SmartDashboard.putNumber("encoderIzqAdel ", valorEncoderIzqAdel);
    SmartDashboard.putNumber("encoderDerAdel ", valorEncoderDerAdel);*/

    /*distanciaRecorrida = valorEncoderDerAdel * (Math.PI * diametroLlanta / relacionUsada)/10;
    SmartDashboard.putNumber("distancia Recorrida ", distanciaRecorrida);*/
      
    

      
    valorEncoderCajaAdel = -encoderCajaShootAdel.getPosition();
    valorEncoderCajaAtras = -encoderCajaShootAtras.getPosition();

    SmartDashboard.putNumber("valorEncoderCajaAdel ", valorEncoderCajaAdel);
    SmartDashboard.putNumber("valorEncoderCajaAtras ", valorEncoderCajaAtras);

  }
/*----###################################################################################################3-- */
  @Override
  public void autonomousInit() {
    resetEncodersChasis();
    resetGyro();
    pidDer.setP(0.0005);
    pidIzq.setP(0.0005);
    pidDer.setI(0.000001);
    relacionUsada = relacionBaja;
    encoderCajaShootAdel.setPosition(0);
    encoderCajaShootAtras.setPosition(0);

    tiempo.start();
    tiempo.reset();

    velocidades.set(true);
  }


  @Override
  public void autonomousPeriodic() {
    //false en alta
    //true en baja

    SmartDashboard.putBoolean("Baja", velocidad);
    SmartDashboard.putBoolean("Alta", !velocidad);
    valorEncoderIzqAdel = encoderIzqAdel.getPosition();
    valorEncoderDerAdel = -encoderDerAdel.getPosition();
    SmartDashboard.putNumber("encoderIzqAdel ", valorEncoderIzqAdel);
    SmartDashboard.putNumber("encoderDerAdel ", valorEncoderDerAdel);

    distanciaRecorrida = valorEncoderIzqAdel * (Math.PI * diametroLlanta / relacionUsada)/10;
    
    SmartDashboard.putNumber("distancia", distanciaRecorrida);
    SmartDashboard.putNumber("getGyro", getGyro());

    SmartDashboard.putNumber("TIEMPO", tiempo.get());

    SmartDashboard.putNumber("angulo ", NavX.getRawGyroY());
   //---------------
   //Auto PARA ADOCKED
   /*double velAutonomo = 0.35;
   if( tiempo.get() <= 2 ){
       //se tira el cubo a 3
   if(valorEncoderCajaAdel <= 0.25){
     velocidad = false;
     motoresCajaShooter.set(-0.04);
   } else if(valorEncoderCajaAdel >= 0.6 && valorEncoderCajaAdel <= 2){
     velocidad = false;
     motoresCajaShooter.set(0.035);
   } else{ 
     motoresCajaShooter.set(0.017);
     velocidad = true;
   }

   if(tiempo.get() > 0 && tiempo.get() < 1 )
   {
     motoresExternosShooter.set(-0.3);
     motorInternoShooterDer.set(0);
   } else if(tiempo.get() >= 1 && tiempo.get() < 1.8){
     motoresExternosShooter.set(-0.3);
     motorInternoShooterDer.set(-0.2); 
     

   }else if(tiempo.get() >= 1.8){
     motoresExternosShooter.set(0);
     motorInternoShooterDer.set(0);
     movimiento1 = true;
   }

   

   } else if( movimiento1 == true && tiempo.get() > 2  && tiempo.get() < 15){
   

    if(distanciaRecorrida  >= -2.3){
     //avanzaa derecho
     if (getGyro() < -2.5 ){
       chasis.tankDrive(0.72, 0.75);
       //chasis.tankDrive(0, 0);
      } else if (getGyro() > 2.5 ){
       chasis.tankDrive(0.75, 0.72);
       //chasis.tankDrive(0, 0);
      } else if(getGyro() >= -2.5 && getGyro()<= 2.5){
       chasis.tankDrive(0.75, 0.77);
      } 
     }
     else{
       chasis.tankDrive(0.0, 0);
       movimiento2 = true;
     }

   }  else {
       chasis.tankDrive(0.0, 0);
       motorInternoShooterDer.set(0);
       motoresExternosShooter.set(0);

     }*/


   //-------------------
    // velocidad positiva hacia atras
    // velocidad negativa hacia adelante

    
     //velAutonomo = 0.35;
    
    //AUTONOMO TIRA, SALE Y GIRA
    /*double velAutonomo = 0.35;
    if( tiempo.get() <= 2 ){
        //se tira el cubo a 3
    if(valorEncoderCajaAdel <= 0.25){
      velocidad = false;
      motoresCajaShooter.set(-0.04);
    } else if(valorEncoderCajaAdel >= 0.6 && valorEncoderCajaAdel <= 2){
      velocidad = false;
      motoresCajaShooter.set(0.035);
    } else{ 
      motoresCajaShooter.set(0.017);
      velocidad = true;
    }

    if(tiempo.get() > 0 && tiempo.get() < 1 )
    {
      motoresExternosShooter.set(-0.35);
      motorInternoShooterDer.set(0);
    } else if(tiempo.get() >= 1 && tiempo.get() < 1.8){
      motoresExternosShooter.set(-0.35);
      motorInternoShooterDer.set(-0.2); 
      

    }else if(tiempo.get() >= 1.8){
      motoresExternosShooter.set(0);
      motorInternoShooterDer.set(0);
      movimiento1 = true;
    }

    

    } else if( movimiento1 == true && tiempo.get() > 2  && tiempo.get() < 7 ){
    

     if(distanciaRecorrida  >= -3){
      //avanzaa derecho
      if (getGyro() < -2.5 ){
        chasis.tankDrive(0.52, 0.55);
        //chasis.tankDrive(0, 0);
       } else if (getGyro() > 2.5 ){
        chasis.tankDrive(0.55, 0.52);
        //chasis.tankDrive(0, 0);
       } else if(getGyro() >= -2.5 && getGyro()<= 2.5){
        chasis.tankDrive(0.55, 0.57);
       } 
      }
      else{
        chasis.tankDrive(0.0, 0);
        movimiento2 = true;
      }

    } else if( tiempo.get() > 7  && tiempo.get() <= 9 ){
      //gira izqu 180  grados
      if(getGyro() >=- 160){
        chasis.tankDrive(0.55, -0.55);
        }else if(getGyro() <= -185){
          chasis.tankDrive(-0.35, .35);
        } else{
          chasis.tankDrive(0, 0);
        } 
    } else {
        chasis.tankDrive(0.0, 0);
        motorInternoShooterDer.set(0);
        motoresExternosShooter.set(0);
 
      }*/
    
   
//---------------------------------
  
/*    PARTE PARA AGARRAR EL CONO  
else if( tiempo.get() > 9  && tiempo.get() <= 9.8 ){
  //bajar disparador
  
  if( valorEncoderCajaAdel <= 3.5){
    motoresCajaShooter.set(-0.08);
  }else {
    motoresCajaShooter.set(0.0);
    //meter
    motorInternoShooterDer.set(0.2);
    motoresExternosShooter.set(0.3);
  }
} else if( tiempo.get() > 9.8  && tiempo.get() <= 10.8 ){
  motorInternoShooterDer.set(0);
  motoresExternosShooter.set(0);
  //avanzaa derecho
  if (getGyro() < -2.5 ){
    chasis.tankDrive(-0.52, -0.55);
    //chasis.tankDrive(0, 0);
   } else if (getGyro() > 2.5 ){
    chasis.tankDrive(-0.55, -0.52);
    //chasis.tankDrive(0, 0);
   } else if(getGyro() >= -2.5 && getGyro()<= 2.5){
    chasis.tankDrive(-0.55, -0.57);
   } 
  } else if( tiempo.get() > 10.8  && tiempo.get() <= 13 ){
    //gira der 0  grados
  if(getGyro() <= -20){
    chasis.tankDrive(-0.55, 0.55);
    }else if(getGyro() >= 5){
      chasis.tankDrive(-0.35, .35);
    } else{
      chasis.tankDrive(0, 0);
    } 
  }  else if( tiempo.get() > 11  && tiempo.get() <= 12 ){
   
  }*/
  
//-------------------------------



/***EXTRA* */
    /*if( tiempo.get() < 4 ){
    if( tiempo.get() > 5 && tiempo.get() < 15){
      if(distanciaRecorrida <= 4){
        chasis.tankDrive(-velAutonomo, -velAutonomo);
      
      } else {
       chasis.tankDrive(0.0, 0.0);
        movimiento1 = true;
        System.out.println("j");
      } 
    } else{
      chasis.tankDrive(0.0, 0.0);
    }

    }*/

    //chasis.tankDrive(0.2, 0.2); 

    
/**** */


    /*//se tira el cubo a 3
    if(valorEncoderCajaAdel <= 0.25){
     velocidad = false;
     motoresCajaShooter.set(-0.04);
    } else if(valorEncoderCajaAdel >= 0.6 && valorEncoderCajaAdel <= 2){
      velocidad = false;
      motoresCajaShooter.set(0.035);
    } else{ 
      motoresCajaShooter.set(0.017);
      velocidad = true;
    }

    if(tiempo.get() > 0 && tiempo.get() < 1 )
      {
        motoresExternosShooter.set(-0.4);
        motorInternoShooterDer.set(0);
      } else if(tiempo.get() > 1 && tiempo.get() < 2.5 ){
        motoresExternosShooter.set(-0.4);
        motorInternoShooterDer.set(-0.2); 
      }else {
        motoresExternosShooter.set(0);
        motorInternoShooterDer.set(0);
      }

      if( tiempo.get() >= 3 && tiempo.get() < 10){
        chasis.tankDrive(0.4, 0.4);
      }*/

  }

  @Override
  public void testInit() {
    resetEncodersChasis();
    relacionUsada = relacionBaja;
    resetGyro();
    velocidades.set(true);
  }

  @Override
  public void testPeriodic() {
    valorEncoderIzqAdel = encoderIzqAdel.getPosition();
    valorEncoderDerAdel = -encoderDerAdel.getPosition();
    SmartDashboard.putNumber("encoderIzqAdel ", valorEncoderIzqAdel);
    SmartDashboard.putNumber("encoderDerAdel ", valorEncoderDerAdel);

    distanciaRecorrida = valorEncoderIzqAdel * (Math.PI * diametroLlanta / relacionUsada)/10;
    
    SmartDashboard.putNumber("distancia", distanciaRecorrida);
    SmartDashboard.putNumber("getGyro", getGyro());

    SmartDashboard.putNumber("TIEMPO", tiempo.get());

    //chasis.tankDrive(0.75*controlDriver.getRawAxis(1), 0.75*controlDriver.getRawAxis(5));

    SmartDashboard.putNumber("jIzquierdo", 0.75*controlDriver.getRawAxis(1));
    SmartDashboard.putNumber("jDerecho    ", 0.75*controlDriver.getRawAxis(5));

    //bajar disparador
    motoresCajaShooter.set(-0.08);
    if( valorEncoderCajaAdel <= 3.5){
      motoresCajaShooter.set(-0.05);
    }else {
      motoresCajaShooter.set(0.0);
      //meter
      motorInternoShooterDer.set(0.2);
      motoresExternosShooter.set(0.3);
    }

    //--------------------------
//GIRO izquierda A 180 5 SEGUNDOs
/*if(getGyro() >=- 165){
  chasis.tankDrive(0.35, -0.35);
  }else if(getGyro() <= -185){
    chasis.tankDrive(-0.2, .2);
  } else{
    chasis.tankDrive(0, 0);
  }*/





//-----------------------

   /*if(distanciaRecorrida  >= -1.0){
      chasis.tankDrive(0.4, 0.4);
      }
      else{
        chasis.tankDrive(0.0, 0);
        movimiento2 = true;
      }*/
      /*if (getGyro() < -2.5 ){
        chasis.tankDrive(0.37, 0.4);
        //chasis.tankDrive(0, 0);
       } else if (getGyro() > 2.5 ){
        chasis.tankDrive(0.4, 0.37);
        //chasis.tankDrive(0, 0);
       } else if(getGyro() >= -2.5 && getGyro()<= 2.5){
        chasis.tankDrive(0.4, 0.42);
       } */

       /*if(getGyro() >=- 175){
        chasis.tankDrive(0.35, -0.35);
        }else if(getGyro() <= -185){
          chasis.tankDrive(-0.2, .2);
        } else{
          chasis.tankDrive(0, 0);
        }*/

        
  }


  /*********************** */

  /*----###################################################################################################3-- */
  @Override
  public void teleopInit() {
    //colocar los resets de abajo para pruebas, en competencia quitarlos
    /*resetEncodersChasis();
    resetGyro();
    encoderCajaShootAdel.setPosition(0);
    encoderCajaShootAtras.setPosition(0);*/
    encoderCajaShootAdel.setPosition(0);
    encoderCajaShootAtras.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {

    /*SmartDashboard.putNumber("getGyro", getGyro());
    SmartDashboard.putNumber("NavX.getAngle", NavX.getAngle());
    SmartDashboard.putNumber("NavX.getYaw", NavX.getYaw());*/

    SmartDashboard.putBoolean("Baja", velocidad);
    SmartDashboard.putBoolean("Alta", !velocidad);

    /*==============================Control Driver========================================== */
    
    if(controlDriver.getRawButton(1))
      tank = true;
    else if(controlDriver.getRawButton(2))
      tank = false;

    if(tank == true)
    {
      chasis.tankDrive(0.75*controlDriver.getRawAxis(1), 0.75*controlDriver.getRawAxis(5));
      SmartDashboard.putNumber("lado izq", 0.75*controlDriver.getRawAxis(1));
      SmartDashboard.putNumber("lado der", 0.75*controlDriver.getRawAxis(5));
    }
    else if(tank == false)
    {
      chasis.arcadeDrive(0.75*controlDriver.getRawAxis(1), 0.75*controlDriver.getRawAxis(4));
      SmartDashboard.putNumber("enfrente/atras", 0.75*controlDriver.getRawAxis(1));
      SmartDashboard.putNumber("giro joystick", 0.75*controlDriver.getRawAxis(4));
    } 

    if(controlDriver.getPOV() == 90){//alta
      velocidades.set(false);
      relacionUsada = relacionAlta;
      velocidad = false;
    }
    if(controlDriver.getPOV() == 270){//baja
      
      velocidades.set(true);
      relacionUsada = relacionBaja;
      velocidad = true;
    }


    /*==============================Control Placer========================================== */

    //CONTROL SHOOTER
    //motoresCajaShooter.set(-0.2*controlPlacer.getRawAxis(5));
    if(controlPlacer.getRawButton(1)){
      //disparador a 70 grados
      if(valorEncoderCajaAdel <= 0.3){
        //se detiene a 0.3
       velocidad = false;
       motoresCajaShooter.set(-0.04);
      } else if(valorEncoderCajaAdel >= 0.6 && valorEncoderCajaAdel <= 2){
        velocidad = false;
        motoresCajaShooter.set(0.035);
      } else{ 
        motoresCajaShooter.set(0.017);
        velocidad = true;
      }
    } else if(controlPlacer.getRawButton(2)){
      //disparador a 90 grados
      if( valorEncoderCajaAdel >= 0.10){
        motoresCajaShooter.set(0.13);
      }else {
        motoresCajaShooter.set(0.0);
      }
    } else if(controlPlacer.getRawButton(3)){
      //bajar disparador
      //motoresCajaShooter.set(-0.05);
      if( valorEncoderCajaAdel <= 3.5){
        motoresCajaShooter.set(-0.05);
      }else {
        motoresCajaShooter.set(0.0);
      }
    }

    if(controlPlacer.getRawButtonPressed(4)==true){
      velocidadShooter = 0.15;
      tiempo.start();
      tiempo.reset();
    }else if(controlPlacer.getRawButtonReleased(4) == true) {
      tiempo.stop();
    } else if(controlPlacer.getRawButton(4)){
      tiempo_activo = true;
    } else if(controlPlacer.getRawButtonPressed(5)==true){
      velocidadShooter = 0.3;
      tiempo.start();
      tiempo.reset();
    }else if(controlPlacer.getRawButtonReleased(5) == true) {
      tiempo.stop();
    } else if(controlPlacer.getRawButton(5)){
      tiempo_activo = true;
    } else if(controlPlacer.getRawButtonPressed(6)==true){
      velocidadShooter = 0.4;
      tiempo.start();
      tiempo.reset();
    }else if(controlPlacer.getRawButtonReleased(6) == true) {
      tiempo.stop();
    } else if(controlPlacer.getRawButton(6)){
      tiempo_activo = true;
    } else if(controlPlacer.getRawButtonPressed(7)==true){
      velocidadShooter = 0.5;
      tiempo.start();
      tiempo.reset();
    }else if(controlPlacer.getRawButtonReleased(7) == true) {
      tiempo.stop();
    } else if(controlPlacer.getRawButton(7)){
      tiempo_activo = true;
    }  else if(controlPlacer.getRawButtonPressed(8)==true){
      velocidadShooter = 0.6;
      tiempo.start();
      tiempo.reset();
    }else if(controlPlacer.getRawButtonReleased(8) == true) {
      tiempo.stop();
    } else if(controlPlacer.getRawButton(8)){
      tiempo_activo = true;
    }else{
      tiempo_activo = false;
    }
    
    if(controlPlacer.getRawAxis(2) >= 0.3){
      //meter
      motorInternoShooterDer.set(0.2);
      motoresExternosShooter.set(0.3);
    }else if(controlPlacer.getPOV() == 0){
      motorInternoShooterDer.set(0.3);
    } else if(controlPlacer.getPOV() == 90){
      motorInternoShooterDer.set(-0.3);
    }else if(controlPlacer.getPOV() == 270){
      motorInternoShooterDer.set(-0.4);
    } else if(controlPlacer.getPOV() == 180){
      motorInternoShooterDer.set(-0.2);
    } else if(controlPlacer.getRawAxis(2) >= 0.3){
      motorInternoShooterDer.set(0.1);
    }
    else {
      if(tiempo.get() > 0 && tiempo.get() < 0.075 && tiempo_activo == true)
      {
        motoresExternosShooter.set(-velocidadShooter);
        motorInternoShooterDer.set(0);
      } else if(tiempo.get() > 0.075 && tiempo_activo == true){
        motoresExternosShooter.set(-velocidadShooter);
        motorInternoShooterDer.set(-0.25);
      }else if (tiempo_activo == false){
        motoresExternosShooter.set(0);
        motorInternoShooterDer.set(0);
      }
    }

    //motorBrazo.set(controlPlacer.getRawAxis(1));
    
  }
/*----###################################################################################################3-- */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

/********************* */

  //Funcion para sacar el promedio de dos encoders
 

  //Reset Encoder Values
  public void resetEncodersChasis(){
    encoderIzqAdel.setPosition(0);
    encoderIzqAtras.setPosition(0);
    encoderDerAdel.setPosition(0);
    encoderDerAtras.setPosition(0);
  }

  //Sets gyroscope yaw to 0
  public void resetGyro(){
    NavX.zeroYaw();
  }

  /*
   * Returns gyroscope Yaw Heading 
   * @return double angle heading
   */
  public double getGyro(){
    return NavX.getAngle();
  }



}
