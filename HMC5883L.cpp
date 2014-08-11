/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 http://c48754.r54.cf3.rackcdn.com/HMC5883L.pdf

*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "includes.h"
#include "HMC5883L.h"

//------------------------------------------------------------------------------
HMC5883L::HMC5883L()
{
	m_Scale = 1;
}

//------------------------------------------------------------------------------
void HMC5883L::Init()
{
	// We're using wiringPi here on an RPi, we need to init the library for each i2c device

	if( (i2c_fd = wiringPiI2CSetup( HMC5883L_Address )) < 0)
	{
		fprintf (stderr, "Unable to open HMC5883L compass: %s\n", strerror (errno)) ;
		return;
	}

	delay(5); // you need to wait at least 5ms after power on to initialize
}

//------------------------------------------------------------------------------
MagnetometerRaw HMC5883L::ReadRawAxis()
{
	Read(DataRegisterBegin, 6);

	MagnetometerRaw raw = MagnetometerRaw();

	raw.XAxis = (buffer[0] << 8) | buffer[1];
	raw.ZAxis = (buffer[2] << 8) | buffer[3];
	raw.YAxis = (buffer[4] << 8) | buffer[5];

	raw.XAxis = signExtened(raw.XAxis);
	raw.YAxis = signExtened(raw.YAxis);
	raw.ZAxis = signExtened(raw.ZAxis);

	return raw;
}

//------------------------------------------------------------------------------
MagnetometerScaled HMC5883L::ReadScaledAxis()
{
	MagnetometerRaw raw = ReadRawAxis();

	MagnetometerScaled scaled = MagnetometerScaled();

	scaled.XAxis = raw.XAxis * m_Scale;
	scaled.ZAxis = raw.ZAxis * m_Scale;
	scaled.YAxis = raw.YAxis * m_Scale;

	return scaled;
}

//------------------------------------------------------------------------------
int HMC5883L::SetScale(float gauss)
{
	U8 regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == 1.3)
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	else
		return ErrorCode_1_Num;
	
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	Write(ConfigurationRegisterB, regValue);
}

//------------------------------------------------------------------------------
int HMC5883L::SetMeasurementMode(U8 mode)
{
	Write(ModeRegister, mode);
}

//------------------------------------------------------------------------------
U8 HMC5883L::EnsureConnected()
{
	U8 data;
	
	Read(IdentityRegister, 1);

	data = buffer[0];

	if(data == IdentityRegisterValue)
		IsConnected = 1;
	else
		IsConnected = 0;

	return IsConnected;
}

//------------------------------------------------------------------------------
void HMC5883L::Write(int address, int data)
{
	wiringPiI2CWriteReg8( i2c_fd, address, data );
}

//------------------------------------------------------------------------------
void HMC5883L::Read(int address, int length)
{
	int i = 0;

	for(i=0; i<length; i++)
	{
		buffer[i] = wiringPiI2CReadReg8( i2c_fd, address++ );
	}
}

//------------------------------------------------------------------------------
//
// signExtened 
//	sign-extends the 16-bit value from the compass to the RPi's 32-bit int size
//
int HMC5883L::signExtened(int value) 
{
    int new_value = (0x0000FFFF & value );
    int mask = 0x00008000;

    if( mask & value )
	{
        new_value += 0xFFFF0000;
    }

    return new_value;
}

