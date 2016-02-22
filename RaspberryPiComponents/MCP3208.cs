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
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Spi;

namespace HCComponentsForPi
{
    public class MCP3208
    {
        //Constants for SPI controller
        private const string SPI_CONTROLLER_NAME = "SPI0";  /* For Raspberry Pi 2, use SPI0                             */
        private const Int32 SPI_CHIP_SELECT_LINE = 0;       /* Line 0 maps to physical pin number 24 on the Rpi2        */
        private SpiDevice SPIMCP3208; // Connction to MCP3208 A2D convertor

        byte[] readBuffer = new byte[3]; /*this is defined to hold the output data*/
        byte[] writeBuffer = new byte[3] { 0x06, 0x00, 0x00 }; // It is SPI port serial input pin, and is used to load channel configuration data into the device  

        public enum Channel
        {
            CH0,
            CH1,
            CH2,
            CH3,
            CH4,
            CH5,
            CH6,
            CH7
        };

        public MCP3208()
        {

        }

        public async Task<bool> Init()
        {
            return await initSpi();

        }

        /* Initialize the SPI bus */
        private async Task<bool> initSpi()
        {
            try
            {
                var settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE); /* Create SPI initialization settings                               */
                settings.ClockFrequency = 10000000;                             /* Datasheet specifies maximum SPI clock frequency of 10MHz         */
                settings.Mode = SpiMode.Mode3;                                  /* The display expects an idle-high clock polarity, we use Mode3    
                                                                                 * to set the clock polarity and phase to: CPOL = 1, CPHA = 1         
                                                                                 */

                string spiAqs = SpiDevice.GetDeviceSelector(SPI_CONTROLLER_NAME);       /* Find the selector string for the SPI bus controller          */
                var devicesInfo = await DeviceInformation.FindAllAsync(spiAqs);         /* Find the SPI bus controller device with our selector string  */
                SPIMCP3208 = await SpiDevice.FromIdAsync(devicesInfo[0].Id, settings);  /* Create an SpiDevice with our bus controller and SPI settings */
            }
            /* If initialization fails, display the exception and stop running */
            catch (Exception ex)
            {
                Debug.WriteLine(ex.ToString());
                //throw new Exception("SPI Initialization Failed", ex);
                return false;
            }

            return true;
        }

        private int convertToInt(byte[] data)
        {
            int result = data[1] & 0x0F;
            result <<= 8;
            result += data[2];

            return result;
        }

        public int ReadChannel(Channel channel)
        {
            int output = 0;

            switch (channel)
            {
                case Channel.CH0:
                    writeBuffer[0] = 0x06; // set to channel 0  
                    writeBuffer[1] = 0x00; // set to channel 0  
                    break;
                case Channel.CH1:
                    writeBuffer[0] = 0x06; // set to channel 1  
                    writeBuffer[1] = 0x40; // set to channel 1  
                    break;
                case Channel.CH2:
                    writeBuffer[0] = 0x06; // set to channel 1  
                    writeBuffer[1] = 0x80; // set to channel 1  
                    break;
                case Channel.CH3:
                    writeBuffer[0] = 0x06; // set to channel 1  
                    writeBuffer[1] = 0xc0; // set to channel 1  
                    break;
                case Channel.CH4:
                    writeBuffer[0] = 0x07; // set to channel 1  
                    writeBuffer[1] = 0x00; // set to channel 1  
                    break;
                case Channel.CH5:
                    writeBuffer[0] = 0x07; // set to channel 1  
                    writeBuffer[1] = 0x40; // set to channel 1  
                    break;
                case Channel.CH6:
                    writeBuffer[0] = 0x07; // set to channel 1  
                    writeBuffer[1] = 0x80; // set to channel 1  
                    break;
                case Channel.CH7:
                    writeBuffer[0] = 0x07; // set to channel 1  
                    writeBuffer[1] = 0xc0; // set to channel 1  
                    break;
            }

            SPIMCP3208.TransferFullDuplex(writeBuffer, readBuffer);
            output = convertToInt(readBuffer);

            return output;
        }

        public double ConvertToVolts(double sourceVoltage, int channelValue)
        {
            return channelValue * (sourceVoltage / 4095);
        }

        public string GetWiringInfo()
        {
            string output = string.Empty;
            output = "MCP3208 8 Channel 12-bit Analog to Digital Convertor" + Environment.NewLine;
            output += "PINS: " + Environment.NewLine;
            output += "   1 - CH0" + Environment.NewLine;
            output += "   2 - CH1" + Environment.NewLine;
            output += "   3 - CH2" + Environment.NewLine;
            output += "   4 - CH3" + Environment.NewLine;
            output += "   5 - CH4" + Environment.NewLine;
            output += "   6 - CH5" + Environment.NewLine;
            output += "   7 - CH6" + Environment.NewLine;
            output += "   8 - CH7" + Environment.NewLine;
            output += "   9 - DGND" + Environment.NewLine;
            output += "   10 - CS/SHDN" + Environment.NewLine;
            output += "   11 - Din" + Environment.NewLine;
            output += "   12 - Dout" + Environment.NewLine;
            output += "   13 - CLK" + Environment.NewLine;
            output += "   14 - AGND" + Environment.NewLine;
            output += "   15 - Vref" + Environment.NewLine;
            output += "   16 - Vdd" + Environment.NewLine + Environment.NewLine;
            output += "RASPBERRY PI 2 REV B WIRING: " + Environment.NewLine;
            output += "9 - GND" + Environment.NewLine;
            output += "10 - GPIO 8 (CE0)" + Environment.NewLine;
            output += "11 - GPIO 10 (MOSI)" + Environment.NewLine;
            output += "12 - GPIO 9 (MISO)" + Environment.NewLine;
            output += "13 - GPIO 11 (SCLK)" + Environment.NewLine;
            output += "14 - GND" + Environment.NewLine;
            output += "15 - 3.3V" + Environment.NewLine;
            output += "16 - 3.3V ";
            return output;
        }

        public void Dispose()
        {
            SPIMCP3208.Dispose();
        }
    }
}
