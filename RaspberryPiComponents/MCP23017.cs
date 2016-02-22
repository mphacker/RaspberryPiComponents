/*
    Raspberry PI Components - Classes to make interfacing with SPI and I2C components easier.
    Copyright (C) 2015 Michael Hacker

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace HCComponentsForPi
{
    public class MCP23017
    {
        // use these constants for controlling how the I2C bus is setup
        private const string I2C_CONTROLLER_NAME = "I2C1"; //specific to RPI2

        private const byte BASE_PORT_EXPANDER_I2C_ADDRESS = 0x20; // 7-bit I2C address of the first port expander

        private const byte PORT_EXPANDER_IODIRA_REGISTER_ADDRESS = 0x00; // IODIR register controls the direction of the GPIO on the port expander
        private const byte PORT_EXPANDER_IODIRB_REGISTER_ADDRESS = 0x01; // IODIR register controls the direction of the GPIO on the port expander

        private const byte PORT_EXPANDER_IPOLA_REGISTER_ADDRESS = 0x02; // Input Polarity Register
        private const byte PORT_EXPANDER_IPOLB_REGISTER_ADDRESS = 0x03; // Input Polarity Register

        private const byte PORT_EXPANDER_GPINTA_ADDRESS = 0x04; // Interrput on Change pin 
        private const byte PORT_EXPANDER_GPINTB_ADDRESS = 0x05; // Interrput on Change pin 

        private const byte PORT_EXPANDER_DEFVALA_ADDRESS = 0x06; // Default Compare Register for Interrupt-on-change
        private const byte PORT_EXPANDER_DEFVALB_ADDRESS = 0x07; // Default Compare Register for Interrupt-on-change

        private const byte PORT_EXPANDER_INTCONA_ADDRESS = 0x08; // Interrupt Control Register
        private const byte PORT_EXPANDER_INTCONB_ADDRESS = 0x09; // Interrupt Control Register

        private const byte PORT_EXPANDER_IOCON_ADDRESS = 0x0A;  // I/O Expander Configruation Register

        private const byte PORT_EXPANDER_GPPUA_ADDRESS = 0x0C; // GPIO Pull-up Resistor Register
        private const byte PORT_EXPANDER_GPPUB_ADDRESS = 0x0D; // GPIO Pull-up Resistor Register

        private const byte PORT_EXPANDER_INTFA_ADDRESS = 0x0E; //Interrupt Flag Register
        private const byte PORT_EXPANDER_INTFB_ADDRESS = 0x0F; //Interrupt Flag Register

        private const byte PORT_EXPANDER_INTCAPA_ADDRESS = 0x10; //Interrupt Capture Register
        private const byte PORT_EXPANDER_INTCAPB_ADDRESS = 0x11; //Interrupt Capture Register

        private const byte PORT_EXPANDER_GPIOA_REGISTER_ADDRESS = 0x12; // GPIO register is used to read the pins input
        private const byte PORT_EXPANDER_GPIOB_REGISTER_ADDRESS = 0x13; // GPIO register is used to read the pins input

        private const byte PORT_EXPANDER_OLATA_REGISTER_ADDRESS = 0x14; // Output Latch register is used to set the pins output high/low
        private const byte PORT_EXPANDER_OLATB_REGISTER_ADDRESS = 0x15; // Output Latch register is used to set the pins output high/low

        private byte PORT_EXPANDER_I2C_ADDRESS = 0x20;

        private byte iodirARegister; // local copy of I2C Port Expander IODIR register
        private byte gpioARegister; // local copy of I2C Port Expander GPIO register
        private byte olatARegister; // local copy of I2C Port Expander OLAT register
        private byte gpintARegister; // local copy of the I2C Port Expander GPINT register
        private byte intconARegister;// local copy of the I2C Port Expander INTCON register

        private byte iodirBRegister; // local copy of I2C Port Expander IODIR register
        private byte gpioBRegister; // local copy of I2C Port Expander GPIO register
        private byte olatBRegister; // local copy of I2C Port Expander OLAT register
        private byte gpintBRegister; // local copy of the I2C Port Expander GPINT register
        private byte intconBRegister;// local copy of the I2C Port Expander INTCON register

        private byte ioconRegister; //local copy of the I2C Port Expander IOCON register

        private I2cDevice i2cPortExpander;
        byte[] i2CWriteBuffer;
        byte[] i2CReadBuffer;
        byte bitMask;

 
        public enum PinValue
        {
            Low,
            High
        }

        public enum PinMode
        {
            Input,
            Ouput
        }

        public enum Pin
        {
            GPA0,
            GPA1,
            GPA2,
            GPA3,
            GPA4,
            GPA5,
            GPA6,
            GPA7,
            GPB0,
            GPB1,
            GPB2,
            GPB3,
            GPB4,
            GPB5,
            GPB6,
            GPB7
        }

        public enum Register
        {
            GPIOA = PORT_EXPANDER_GPIOA_REGISTER_ADDRESS,
            GPIOB = PORT_EXPANDER_GPIOB_REGISTER_ADDRESS,
            IODIRA = PORT_EXPANDER_GPIOA_REGISTER_ADDRESS,
            IODIRB = PORT_EXPANDER_IODIRB_REGISTER_ADDRESS,
            GPINTA = PORT_EXPANDER_GPINTA_ADDRESS,
            GPINTB = PORT_EXPANDER_GPINTB_ADDRESS,
            IPOLA = PORT_EXPANDER_IPOLA_REGISTER_ADDRESS,
            IPOLB = PORT_EXPANDER_IPOLB_REGISTER_ADDRESS,
            DEFVALA = PORT_EXPANDER_DEFVALA_ADDRESS,
            DEFVALB = PORT_EXPANDER_DEFVALB_ADDRESS,
            INTCON = PORT_EXPANDER_IOCON_ADDRESS,
            GPPUA = PORT_EXPANDER_GPPUA_ADDRESS,
            GPPUB = PORT_EXPANDER_GPPUB_ADDRESS,
            INTFA = PORT_EXPANDER_INTFA_ADDRESS,
            INTFB = PORT_EXPANDER_INTFB_ADDRESS,
            INTCAPA = PORT_EXPANDER_INTCAPA_ADDRESS,
            INTCAPB = PORT_EXPANDER_INTCAPB_ADDRESS,
            OLATA = PORT_EXPANDER_OLATA_REGISTER_ADDRESS,
            OLATB = PORT_EXPANDER_OLATB_REGISTER_ADDRESS
        }


        public MCP23017()
        {
            PORT_EXPANDER_I2C_ADDRESS = BASE_PORT_EXPANDER_I2C_ADDRESS;
        }


        //Index of the device we are accessing. Index 0 is the first device
        public MCP23017(int deviceIndex)
        {
            if (deviceIndex > 0)
                PORT_EXPANDER_I2C_ADDRESS = (byte)(BASE_PORT_EXPANDER_I2C_ADDRESS + deviceIndex);
            else
                PORT_EXPANDER_I2C_ADDRESS = BASE_PORT_EXPANDER_I2C_ADDRESS;
        }

        public async Task<bool> Init()
        {
           return await init();
        }

        private async Task<bool> init()
        {

            // initialize I2C communications
            string deviceSelector = I2cDevice.GetDeviceSelector();
            var i2cDeviceControllers = await DeviceInformation.FindAllAsync(deviceSelector);
            if (i2cDeviceControllers.Count == 0)
            {
                throw new Exception("No I2C controllers were found on this system.");
            }

            var i2cSettings = new I2cConnectionSettings(PORT_EXPANDER_I2C_ADDRESS);
            i2cSettings.BusSpeed = I2cBusSpeed.FastMode;
            i2cPortExpander = await I2cDevice.FromIdAsync(i2cDeviceControllers[0].Id, i2cSettings);
            if (i2cPortExpander == null)
            {
                throw new Exception(string.Format(
                    "Slave address {0} is currently in use on {1}. " +
                    "Please ensure that no other applications are using I2C.",
                    i2cSettings.SlaveAddress,
                    i2cDeviceControllers[0].Id));
            }

            // initialize I2C Port Expander registers
            try
            {
                GetRegisterValues();

                //Disable pull up resistor
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_GPPUA_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                i2CWriteBuffer = new byte[] { PORT_EXPANDER_GPPUB_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                //Set all to outputs
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IODIRA_REGISTER_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IODIRB_REGISTER_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                //reset input polarity
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IPOLA_REGISTER_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IPOLB_REGISTER_ADDRESS, 0x00 };
                i2cPortExpander.Write(i2CWriteBuffer);

                //configure device for open-drain output on the interrupt pin
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IOCON_ADDRESS, 0x02 };
                i2cPortExpander.Write(i2CWriteBuffer);

            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.ToString());
                return false;
                //throw new Exception("Failed to initialize I2C port expander: " + e.Message);
            }
            return true;
        }

        private void GetRegisterValues()
        {
            // initialize local copies of the IODIR, GPIO, and OLAT registers
            i2CReadBuffer = new byte[1];

            // read in each register value on register at a time (could do this all at once but
            // for example clarity purposes we do it this way)
            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_IODIRA_REGISTER_ADDRESS }, i2CReadBuffer);
            iodirARegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_GPIOA_REGISTER_ADDRESS }, i2CReadBuffer);
            gpioARegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_OLATA_REGISTER_ADDRESS }, i2CReadBuffer);
            olatARegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_IODIRB_REGISTER_ADDRESS }, i2CReadBuffer);
            iodirBRegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_GPIOB_REGISTER_ADDRESS }, i2CReadBuffer);
            gpioBRegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_OLATB_REGISTER_ADDRESS }, i2CReadBuffer);
            olatBRegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_GPINTA_ADDRESS }, i2CReadBuffer);
            gpintARegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_GPINTB_ADDRESS }, i2CReadBuffer);
            gpintBRegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_INTCONA_ADDRESS }, i2CReadBuffer);
            intconARegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_INTCONB_ADDRESS }, i2CReadBuffer);
            intconBRegister = i2CReadBuffer[0];

            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_IOCON_ADDRESS}, i2CReadBuffer);
            ioconRegister = i2CReadBuffer[0];

        }

        public void SetDriveMode(Pin pin, PinMode pinMode)
        {
            byte PORT_EXPANDER_OLAT_REGISTER_ADDRESS = 0x00;
            byte PORT_EXPANDER_IODIR_REGISTER_ADDRESS = 0x00;
            byte olatRegister = 0x00;
            byte iodirRegister = 0x00;

            string pinName = pin.ToString();
            GetRegisterValues();
            if (pinName.Substring(2, 1) == "A")
            {
                PORT_EXPANDER_OLAT_REGISTER_ADDRESS = PORT_EXPANDER_OLATA_REGISTER_ADDRESS;
                PORT_EXPANDER_IODIR_REGISTER_ADDRESS = PORT_EXPANDER_IODIRA_REGISTER_ADDRESS;
                olatRegister = olatARegister;
                iodirRegister = iodirARegister;

            }

            if (pinName.Substring(2, 1) == "B")
            {
                PORT_EXPANDER_OLAT_REGISTER_ADDRESS = PORT_EXPANDER_OLATB_REGISTER_ADDRESS;
                PORT_EXPANDER_IODIR_REGISTER_ADDRESS = PORT_EXPANDER_IODIRB_REGISTER_ADDRESS;
                olatRegister = olatBRegister;
                iodirRegister = iodirBRegister;
            }

            Byte pinAddress = getPinAddress(pinName);

            if (pinMode == PinMode.Ouput)
            {

                //// configure the pin output to be logic low, leave the other pins as they are.
                bitMask = (byte)(0xFF ^ pinAddress);
                olatRegister &= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_OLAT_REGISTER_ADDRESS, olatRegister };
                i2cPortExpander.Write(i2CWriteBuffer);

                //// configure only the pin to be an output and leave the other pins as they are.
                //// input is logic high, output is logic low
                bitMask = (byte)(0xFF ^ pinAddress);
                iodirRegister &= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IODIR_REGISTER_ADDRESS, iodirRegister };
                i2cPortExpander.Write(i2CWriteBuffer);
            }

            if (pinMode == PinMode.Input)
            {
                bitMask = (byte)(0x00 ^ pinAddress);
                iodirRegister |= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_IODIR_REGISTER_ADDRESS, iodirRegister };
                i2cPortExpander.Write(i2CWriteBuffer);
            }
        }

        public List<Pin> GetChangedPins()
        {
            List<Pin> output = new List<Pin>();
            byte changedPinsA = 0x00;
            byte changedPinsB = 0x00;
            string binary = string.Empty;

            i2CReadBuffer = new byte[1];
            //Check A PINs
            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_INTFA_ADDRESS }, i2CReadBuffer);
            changedPinsA = i2CReadBuffer[0];
            binary = Convert.ToString(changedPinsA, 2).PadLeft(8,'0');
            if (changedPinsA>0)
            {
                if (binary.Substring(0, 1) == "1")
                    output.Add(Pin.GPA7);
                if (binary.Substring(1, 1) == "1")
                    output.Add(Pin.GPA6);
                if (binary.Substring(2, 1) == "1")
                    output.Add(Pin.GPA5);
                if (binary.Substring(3, 1) == "1")
                    output.Add(Pin.GPA4);
                if (binary.Substring(4, 1) == "1")
                    output.Add(Pin.GPA3);
                if (binary.Substring(5, 1) == "1")
                    output.Add(Pin.GPA2);
                if (binary.Substring(6, 1) == "1")
                    output.Add(Pin.GPA1);
                if (binary.Substring(7, 1) == "1")
                    output.Add(Pin.GPA0);
            }

            //Check B PINS
            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_INTFB_ADDRESS }, i2CReadBuffer);
            changedPinsB = i2CReadBuffer[0];
            binary = Convert.ToString(changedPinsB, 2).PadLeft(8,'0');
            if (changedPinsB>0)
            {
                if (binary.Substring(0, 1) == "1")
                    output.Add(Pin.GPB7);
                if (binary.Substring(1, 1) == "1")
                    output.Add(Pin.GPB6);
                if (binary.Substring(2, 1) == "1")
                    output.Add(Pin.GPB5);
                if (binary.Substring(3, 1) == "1")
                    output.Add(Pin.GPB4);
                if (binary.Substring(4, 1) == "1")
                    output.Add(Pin.GPB3);
                if (binary.Substring(5, 1) == "1")
                    output.Add(Pin.GPB2);
                if (binary.Substring(6, 1) == "1")
                    output.Add(Pin.GPB1);
                if (binary.Substring(7, 1) == "1")
                    output.Add(Pin.GPB0);
            }

            return output;
        }

        public void SetInterruptOnChange(Pin pin, bool setInterruptOn)
        {
            byte PORT_EXPANDER_INTERRUPT_ON_CHANGE_ADDRESS = 0x00;
            byte PORT_EXPANDER_INTCON_ADDRESS = 0x00;
            byte GPINTRegister = 0x00;
            byte INTCONRegister = 0x00;

            string pinName = pin.ToString();
            GetRegisterValues();

            if (pinName.Substring(2, 1) == "A")
            {
                PORT_EXPANDER_INTERRUPT_ON_CHANGE_ADDRESS = PORT_EXPANDER_GPINTA_ADDRESS;
                PORT_EXPANDER_INTCON_ADDRESS = PORT_EXPANDER_INTCONA_ADDRESS;
                GPINTRegister = gpintARegister;
                INTCONRegister = intconARegister;
            }

            if (pinName.Substring(2, 1) == "B")
            {
                PORT_EXPANDER_INTERRUPT_ON_CHANGE_ADDRESS = PORT_EXPANDER_GPINTB_ADDRESS;
                PORT_EXPANDER_INTCON_ADDRESS = PORT_EXPANDER_INTCONB_ADDRESS;
                GPINTRegister = gpintBRegister;
                INTCONRegister = intconBRegister;
            }
            Byte pinAddress = getPinAddress(pinName);

            if (setInterruptOn==false)
            {
                //// configure only the pin to be an output and leave the other pins as they are.
                //// input is logic high, output is logic low
                bitMask = (byte)(0xFF ^ pinAddress);
                GPINTRegister &= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_INTERRUPT_ON_CHANGE_ADDRESS, GPINTRegister };
                i2cPortExpander.Write(i2CWriteBuffer);
            }

            if (setInterruptOn==true)
            {
                //ENABLE INTERRUPT ON CHANGE FOR PIN
                bitMask = (byte)(0x00 ^ pinAddress);
                GPINTRegister |= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_INTERRUPT_ON_CHANGE_ADDRESS, GPINTRegister };
                i2cPortExpander.Write(i2CWriteBuffer);

                //SET INTCON TO COMPARE GPIO WITH PREVIOUS PIN VALUE
                bitMask = (byte)(0xFF ^ pinAddress);
                INTCONRegister |= bitMask;
                i2CWriteBuffer = new byte[] { PORT_EXPANDER_INTCON_ADDRESS, INTCONRegister };
                i2cPortExpander.Write(i2CWriteBuffer);
            }
        }

        public void Write(Register reg, byte value)
        {
            i2cPortExpander.Write(new byte[] {(byte)reg, value });
        }

        public void Write(Pin pin, PinValue pinValue)
        {
            byte PORT_EXPANDER_OLAT_REGISTER_ADDRESS = 0x00;
            byte olatRegister = 0x00;

            string pinName = pin.ToString();
            if (pinName.Substring(2, 1) == "A")
            {
                PORT_EXPANDER_OLAT_REGISTER_ADDRESS = PORT_EXPANDER_OLATA_REGISTER_ADDRESS;
                olatRegister = olatARegister;
            }

            if (pinName.Substring(2, 1) == "B")
            {
                PORT_EXPANDER_OLAT_REGISTER_ADDRESS = PORT_EXPANDER_OLATB_REGISTER_ADDRESS;
                olatRegister = olatBRegister;
            }

            Byte pinAddress = getPinAddress(pinName);

            if (pinValue == PinValue.High)
            {
                olatRegister |= pinAddress;
                i2cPortExpander.Write(new byte[] { PORT_EXPANDER_OLAT_REGISTER_ADDRESS, olatRegister });
            }
            if (pinValue == PinValue.Low)
            {
                bitMask = (byte)(0xFF ^ pinAddress);
                olatRegister &= bitMask;
                i2cPortExpander.Write(new byte[] { PORT_EXPANDER_OLAT_REGISTER_ADDRESS, olatRegister });
            }

        }

        public bool Read(Pin pin)
        {
            bool pinStatus = false;
            byte PORT_EXPANDER_GPIO_REGISTER_ADDRESS = 0x00;

            string pinName = pin.ToString();
            if (pinName.Substring(2, 1) == "A")
            {
                PORT_EXPANDER_GPIO_REGISTER_ADDRESS = PORT_EXPANDER_GPIOA_REGISTER_ADDRESS;
            }

            if (pinName.Substring(2, 1) == "B")
            {
                PORT_EXPANDER_GPIO_REGISTER_ADDRESS = PORT_EXPANDER_GPIOB_REGISTER_ADDRESS;
            }

            Byte pinAddress = getPinAddress(pinName);
            byte[] readBuffer = new byte[1];
            i2cPortExpander.WriteRead(new byte[] { PORT_EXPANDER_GPIO_REGISTER_ADDRESS }, readBuffer);

            //Check to see if the pin has a 0 value
            if ((byte)(readBuffer[0] & pinAddress) == 0x00)
            {
                pinStatus = false;
            }
            else
            {
                pinStatus = true;
            }

            return pinStatus;
        }

        private byte getPinAddress(string thepin)
        {
            int results = 0;
            int pinnum = Convert.ToInt32(thepin.ToString().Substring(3, 1));

            if (pinnum == 0)
            {
                results = 0x01;
                return (Byte)results;
            }

            results = 1 << pinnum;

            return (Byte)results;
        }

        public string GetWiringInfo()
        {
            string output = string.Empty;
            output = "MCP23017 16-Bit I/O Expander with Serial Interface" + Environment.NewLine;
            output += "PINS: " + Environment.NewLine;
            output += "   1 - GPB0" + Environment.NewLine;
            output += "   2 - GPB1" + Environment.NewLine;
            output += "   3 - GPB2" + Environment.NewLine;
            output += "   4 - GPB3" + Environment.NewLine;
            output += "   5 - GPB4" + Environment.NewLine;
            output += "   6 - GPB5" + Environment.NewLine;
            output += "   7 - GPB6" + Environment.NewLine;
            output += "   8 - GPB7" + Environment.NewLine;
            output += "   9 - Vdd" + Environment.NewLine;
            output += "   10 - Vss" + Environment.NewLine;
            output += "   11 - NC" + Environment.NewLine;
            output += "   12 - SCL" + Environment.NewLine;
            output += "   13 - SDA" + Environment.NewLine;
            output += "   14 - NC" + Environment.NewLine;
            output += "   15 - A0" + Environment.NewLine;
            output += "   16 - A1" + Environment.NewLine;
            output += "   17 - A2" + Environment.NewLine;
            output += "   18 - RESET" + Environment.NewLine;
            output += "   19 - INTB" + Environment.NewLine;
            output += "   20 - INTA" + Environment.NewLine;
            output += "   21 - GPA0" + Environment.NewLine;
            output += "   22 - GPA1" + Environment.NewLine;
            output += "   23 - GPA2" + Environment.NewLine;
            output += "   24 - GPA3" + Environment.NewLine;
            output += "   25 - GPA4" + Environment.NewLine;
            output += "   26 - GPA5" + Environment.NewLine;
            output += "   27 - GPA6" + Environment.NewLine;
            output += "   28 - GPA7" + Environment.NewLine + Environment.NewLine;
            output += "RASPBERRY PI 2 REV B WIRING: " + Environment.NewLine;
            output += "9 - Vcc (3.3v)" + Environment.NewLine;
            output += "10 - GND" + Environment.NewLine;
            output += "12 - GPIO 3 (SCL)" + Environment.NewLine;
            output += "13 - GPIO 2 (SDA)" + Environment.NewLine;
            output += "15 - GND" + Environment.NewLine;
            output += "16 - GND" + Environment.NewLine;
            output += "17 - GND" + Environment.NewLine;
            output += "18 - Vcc (3.3V) " + Environment.NewLine;
            output += "19 - any GPIO pin for monitoring GPBx interrupts. " + Environment.NewLine;
            output += "20 - any GPIO pin for monitoring GPAx interrupts. " + Environment.NewLine + Environment.NewLine;
            output += "Set pins 15, 16, and 17 to either 3.3V or ground to set the hardware address for this chip." + Environment.NewLine;
            output += "The settings shown above would be for a 7 bit hardware address of 0x20.";

            return output;
        }

        public void Dispose()
        {
            i2cPortExpander.Dispose();
        }
    }
}
