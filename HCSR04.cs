using System;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace HCComponentsForPi
{

    /* HC-SR04 Untrasonic Sensor
        •Power Supply :5V DC
        •Quiescent Current : <2mA
        •Effectual Angle: <15°
        •Ranging Distance : 2cm – 500 cm/1" - 16ft
        •Resolution : 0.3 cm
    */
    public class HCSR04
    {
        private GpioPin rpTriggerPin;
        private GpioPin rpEchoPin;
        private MCP23017 expander = null;
        private MCP23017.Pin ioEchoPin = MCP23017.Pin.GPA0;
        private MCP23017.Pin ioTriggerPin = MCP23017.Pin.GPA0;

        //For use with Raspberry Pi built in GPIO pins
        //Expects pins to have already been initialized and set for output.
        public HCSR04(GpioPin TriggerPin, GpioPin EchoPin)
        {
            rpEchoPin = EchoPin;
            rpTriggerPin = TriggerPin;
        }

        //For use with the MCP23017 I/O expander GPIO pins
        public HCSR04(MCP23017 Expander, MCP23017.Pin TriggerPin, MCP23017.Pin EchoPin)
        {
            expander = Expander;
            ioTriggerPin = TriggerPin;
            ioEchoPin = EchoPin;
        }

        public enum Units
        {
            Millimeters,
            Centimeters,
            Inches,
            Feet
        }


        public double GetDistance(Units toUnits)
        {
            double results = 0;
            TimeSpan elapsedTime;

            Stopwatch sw = new Stopwatch();
            Stopwatch sw2 = new Stopwatch();
            sw2.Reset();
            sw2.Start();
            sw.Reset();
            long startTicks = 0;
            long ticksPerSecond = Stopwatch.Frequency;

            //Although the majority of the code between the Raspberry Pi GPIO
            //and the expander I/O will be the same I am separating them to 
            //reduce any additional operations which could alter the timings
            if (expander == null)
            {
                rpTriggerPin.SetDriveMode(GpioPinDriveMode.Output);
                rpEchoPin.SetDriveMode(GpioPinDriveMode.Input);

                //Turn off the trigger
                rpTriggerPin.Write(GpioPinValue.Low);

                // wait for the sensor to settle
                Task.Delay(TimeSpan.FromMilliseconds(500)).Wait();

                // turn on the pulse
                rpTriggerPin.Write(GpioPinValue.High);

                // let the pulse run for 10 microseconds
                Task.Delay(TimeSpan.FromMilliseconds(.01)).Wait();

                // turn off the pulse
                rpTriggerPin.Write(GpioPinValue.Low);

                // start the stopwatch just as the echo starts
                startTicks = sw2.ElapsedTicks;
                while (rpEchoPin.Read() == GpioPinValue.Low && sw2.ElapsedTicks - startTicks < ticksPerSecond) ;

                sw.Start();

                // stop the stopwatch when the echo stops
                startTicks = sw2.ElapsedTicks;
                while (rpEchoPin.Read() == GpioPinValue.High && sw2.ElapsedTicks - startTicks < ticksPerSecond) ;
                sw.Stop();

                // the duration of the echo is equal to the pulse's roundtrip time
                elapsedTime = sw.Elapsed;

            }
            else
            {
                expander.SetDriveMode(ioTriggerPin, MCP23017.PinMode.Ouput);
                expander.SetDriveMode(ioEchoPin, MCP23017.PinMode.Input);

                // turn off the trigger
                expander.Write(ioTriggerPin, MCP23017.PinValue.Low);

                // wait for the sensor to settle
                Task.Delay(TimeSpan.FromMilliseconds(500)).Wait();

                // turn on the pulse
                expander.Write(ioTriggerPin, MCP23017.PinValue.High);

                // let the pulse run for 10 microseconds
                Task.Delay(TimeSpan.FromMilliseconds(.01)).Wait();

                // turn off the pulse
                expander.Write(ioTriggerPin, MCP23017.PinValue.Low);
                startTicks = sw2.ElapsedTicks;

                // start the stopwatch just as the echo starts
                while (expander.Read(ioEchoPin) == false && sw2.ElapsedTicks - startTicks < ticksPerSecond) ;

                sw.Start();
                // stop the stopwatch when the echo stops

                startTicks = sw2.ElapsedTicks;
                while (expander.Read(ioEchoPin) == true && sw2.ElapsedTicks - startTicks < ticksPerSecond) ;
                sw.Stop();

                // the duration of the echo is equal to the pulse's roundtrip time
                elapsedTime = sw.Elapsed;
            }

            /*Convert value to proper units
                Speed of sound at ground level in air
                13.51 inches per second
                1.126 ft per second
                34.32 centimeters per second
                343.2 millimeters per second
            */

            //Since the sound is sent out and bounces back we need
            //to take the elapsed time in milliseconds and divide it by 2
            results = elapsedTime.TotalMilliseconds / 2;

            switch (toUnits)
            {
                case Units.Centimeters:
                    results = results * 34.32;
                    break;

                case Units.Millimeters:
                    results = results * 343.2;
                    break;

                case Units.Feet:
                    results = results * 1.126;
                    break;

                case Units.Inches:
                    results = results * 13.51;
                    break;
            }

            return results;
        }

        public string GetWiringInfo()
        {
            string output = string.Empty;
            output = "HC SR04 Ultrasonic Sensor" + Environment.NewLine;
            output += "PINS: " + Environment.NewLine;
            output += "   1 - Vcc (5 volts)" + Environment.NewLine;
            output += "   2 - Trigger" + Environment.NewLine;
            output += "   3 - Echo" + Environment.NewLine;
            output += "   4 - Ground" + Environment.NewLine + Environment.NewLine;
            output += "Due to the timing sensitive nature of this component it is recommended that it is connected directly to GPIO ports on the Rasperry Pi 2 and not to an I/O expansion chip.  Connecting to an I/O expansion chip will result in less accurate measurements." + Environment.NewLine + Environment.NewLine;
            output += "Wiring to the Raspberry Pi 2" + Environment.NewLine;
            output += "1 to 5V" + Environment.NewLine;
            output += "2 to GPIO output port" + Environment.NewLine;
            output += "3 to 680 ohm resistor then to GPIO input port then to 1K ohm resistor to ground.  (need resistors to drop voltage to GPIO to under 3.3v and keep current low.  Also protects in case GPIO pin is incorrectly set as an output)" + Environment.NewLine;
            output += "4 to ground" + Environment.NewLine;

            return output;
        }
    }
}
