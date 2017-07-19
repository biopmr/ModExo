//**************************************************************************
//------- Laboratorio de Biomecatronica @ Universidade de Sao Paulo --------
//
// Version: 1.0
// Date: 15.06.2017 [mm.dd.yyyy]
// Description
//  CAN Communication between Arduino and EPOS2 for Exoskeleton Control
// Input files:
// (None)
//
// Final Output files:
//   (None)
//
// References
//  http://wiki.seeed.cc/CAN-BUS_Shield_V1.2/
//  http://www.instructables.com/id/Arduino-Timer-Interrupts/
//  http://forum.arduino.cc/index.php?topic=225789.0
//  
//**************************************************************************
// Steps
//**************************************************************************

#include "SPI.h"
#include "mcp_can.h"
#include "mcp_can_dfs.h"
#include "Arduino.h"

// Define States
#define Startup                           1
#define PDOConfiguration                  2
#define Operational                       3
#define OperationalEncoderControl         4
#define SetPreOperational                 5
#define OperationalDifferentialControl    6 
#define SetOperational                    7
#define GoHome                            8

// Statemachine State variable and initial value
byte State;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN;
// Set CS pin
MCP_CAN CAN(SPI_CS_PIN); 

// Sensor variables
uint32_t encoder_data;
uint32_t current_data0;
uint32_t actualposition_data;
int32_t loadcell_data;
double loadcell_data_double;
bool sync_flag; 
double user_setpoint;
double targetposition;

// ***************************
// EPOS2 CANOpen Communication
// ***************************
//

// Statemachine
unsigned char set_preoperational[2];
unsigned char set_operational[2];
unsigned char enable_epos[8];
unsigned char disable_epos[8];

// PDO Configuration
unsigned char pdo_sync[1];

unsigned char pdo_actual_position_1[8];
unsigned char pdo_actual_position_2[8];

unsigned char pdo_actual_velocity_1[8];
unsigned char pdo_actual_velocity_2[8];

unsigned char pdo_actual_current_1[8];
unsigned char pdo_actual_current_2[8];
unsigned char pdo_actual_current_3[8];
unsigned char pdo_actual_current_4[8];
unsigned char pdo_actual_current_5[8];

// Parameters
unsigned char set_max_following_error[8];
unsigned char set_max_acceleration[8];
unsigned char set_max_profile_velocity[8];
unsigned char set_min_position_limit[8];
unsigned char set_max_position_limit[8];

// Commanding - Object Writing
unsigned char setpoint_position[8];

// EPOS Sensors - Object Reading
unsigned char get_actual_position[8];
unsigned char get_actual_velocity[8];
unsigned char get_actual_current[8];

// *****************
// System Variables
// *****************

int encoder_resolution; //2000 counts per turn
int reduction; // harmonic drive reduction

int CT_K; // generic constant to be tweaked during runtime

// ******************
// System Parameters
// ******************

double x_1;
double x_2;
double x_3;
unsigned long dt0;
unsigned long dt1;
unsigned long dt;

// human parameters (plant) {from Olaya cap5}
// long j_h;
// long b_h;
// long k_h;
long j_h;
long b_h;
long k_h;

// Thus our values for the parameters for the exoskeleton mechanism were Ie = 0.199 kg-m, be = 1.32 N-m-s/rad, ke = 5.12 N-m/rad
long j_exo; // Nms^2/rad
long b_exo; // Nms/rad [2.5-4.7]
long k_exo; // Nm/rad [5.3-13.1]

// coupled system (plant)
long j_eq; // Nms^2/rad
long b_eq; // Nms/rad 
long k_eq; // Nm/rad

// ******************
// LoadCell
// ******************

// [[loadcell_data_double = A*(force) + B]]
float A; //slop steepness
// float B = 118712.7; //offset
float B; //offset
float d; ////distancia entre os centros dos aros da celula de carga=10,5cm
double contactForce;
double contactTorque;

//********************
// EPOS COMMUNICATION
//********************
//-----------------------------------------------------------------
void doStartup(void);
//-----------------------------------------------------------------
void PDOConfig(void);
//-----------------------------------------------------------------
void doHoming(void);
//-----------------------------------------------------------------
void positionSetpoint(uint32_t angle);
//-----------------------------------------------------------------
void gotoPositionZero();
//-----------------------------------------------------------------
void sync(void);

//********************
// DATA AQUISITION
//********************
//-----------------------------------------------------------------
float amplificationBoardDataRead();

//***********************
// INTERFACE
//***********************
//-----------------------------------------------------------------
void DataPrint();
//-----------------------------------------------------------------
int readSerialInteger();
//-----------------------------------------------------------------
void serialController(char command);

//****************
// CONTROL MODES
//****************
//-----------------------------------------------------------------
float EncoderControl();
//-----------------------------------------------------------------
double DifferentialEquation();

//****************
// ARDUINO
//****************
//-----------------------------------------------------------------
void setupModExo();
//-----------------------------------------------------------------
void loopModExo();

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/