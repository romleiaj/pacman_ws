#include <iostream>
#include <stdio.h>
#include <string.h>
#include "roboteq_api/RoboteqDevice.h"
#include "roboteq_api/ErrorCodes.h"
#include "roboteq_api/Constants.h"
using namespace std;
int main(int argc, char *argv[])
{
string response = "";
//Create an instance of RoboteqDevice.
RoboteqDevice device;
//Connect to the device, for windows use "\\\\.\\com1" for com1.
int status = device.Connect("/dev/ttyACM0");
//Check to see if the connection succeeded.
if(status != RQ_SUCCESS)
{
cout<<"Error connecting to device: "<<status<<"."<<endl;
return 1;
}

sleepms(10);
int result;


//Wait 10 ms before sending another command to device
cout<<"- SetConfig(_RWD, 0, 0)...";
if((status = device.SetConfig(_RWD, 0, 0)) != RQ_SUCCESS)
cout<<"failed --> "<<status<<endl;
else
cout<<"succeeded now sleep."<<endl;

//Wait 10 ms before sending another command to device
sleepms(10);
cout<<"- SetCommand(_GO, 1, 500)...";
if((status = device.SetCommand(_GO, 1, -500)) != RQ_SUCCESS)
cout<<"failed --> "<<status<<endl;
else
cout<<"succeeded now sleep."<<endl;

//Wait 10 ms before sending another command to device
sleepms(10);
cout<<"- SetCommand(_GO, 2, 500)...";
if((status = device.SetCommand(_GO, 2, 500)) != RQ_SUCCESS)
cout<<"failed --> "<<status<<endl;
else
cout<<"succeeded now sleep."<<endl;

sleepms(5000);

cout<<"- SetCommand(_GO, 1, 0)...";
if((status = device.SetCommand(_GO, 1, 0)) != RQ_SUCCESS)
cout<<"failed --> "<<status<<endl;
else
cout<<"succeeded."<<endl;
//
sleepms(10);
cout<<"- SetCommand(_GO, 2, 0)...";
if((status = device.SetCommand(_GO, 2, 0)) != RQ_SUCCESS)
cout<<"failed --> "<<status<<endl;
else
cout<<"succeeded."<<endl;

device.Disconnect();
return 0;
}
