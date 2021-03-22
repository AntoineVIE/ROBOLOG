#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float32MultiArray float_msg;
ros::Publisher chatter("chatter", &float_msg);

float Acc_data[2];

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner \n");

  nh.initNode();
  nh.advertise(chatter);
}


void write(int address, byte val)
{
  Wire.beginTransmission(0x69);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

byte read(int address)
{
  Wire.beginTransmission(0x69);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(0x69, 8);
  byte val = Wire.read();
  //Serial.print("La valeur lue est : ");
  //Serial.println(val);

  return val;
}

float MesureAcc(char axe)
{
  write(0x6B, 0);
  write(0x6C, 0);
  byte Power1 = read(0x6B);
  byte Power2 = read(0x6C);
  signed int Acc;
  
  if (axe == 'x')
  {
    byte Acc_X_High = read(0x3B);
    //Serial.println(Acc_X_High,BIN);
    byte Acc_X_Low = read(0x3C);
    //Serial.println(Acc_X_Low,BIN);
    Acc = ((long)Acc_X_High<<8) | (long)Acc_X_Low;
    //Serial.println(Acc,BIN);
  }

  else if (axe == 'y')
  {
    byte Acc_Y_High = read(0x3D);
    //Serial.println(Acc_Y_High,BIN);
    byte Acc_Y_Low = read(0x3E);
    //Serial.println(Acc_Y_Low,BIN);
    Acc = ((long)Acc_Y_High<<8) | (long)Acc_Y_Low;
    //Serial.print("Acc non inversé : ");
    //Serial.println(Acc,BIN);
  }

  else if (axe == 'z')
  {
    byte Acc_Z_High = read(0x3F);
    //Serial.println(Acc_Z_High,BIN);
    byte Acc_Z_Low = read(0x40);
    //Serial.println(Acc_Z_Low,BIN);
    Acc = ((long)Acc_Z_High<<8) | (long)Acc_Z_Low;
    //Serial.println(Acc,BIN);
  }

  else
  {
    Serial.print("Axis not find");
    exit(0);
  }

  Acc = ~Acc + 0x01;
  float Acc_convert = 2*9.81/(32768)*Acc;
  Serial.print("Acc converti : ");
  Serial.println(Acc_convert,DEC);
  return Acc_convert;
}

void loop()
{
  byte error,address;
  int nDevices;

  Serial.println("Scan en cours \n");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("Péripherique I2C à l'adresse 0x");
      Serial.println(address,HEX);
      nDevices++;
    }

    else if (error == 4)
    {
      Serial.print("Erreur à l'adresse 0x");
      Serial.println(address,HEX);
    }
  }

  if (nDevices == 0)
  {
    Serial.println("Aucun périphérique I2C trouvé\n");
  }

  else
  {
    Serial.println("Fin du scan\n");
  }

  float Acc_x = MesureAcc('x');
  float Acc_y = MesureAcc('y');

  float Acc_data[2];

  Acc_data[0] = Acc_x;
  Acc_data[1] = Acc_y;

  Serial.println(Acc_data[0]);
  Serial.println(Acc_data[1]);

  float_msg.data = Acc_data;
  float_msg.data_length = 2;
  chatter.publish(&float_msg);
  nh.spinOnce();
  delay(1000);
 
}
