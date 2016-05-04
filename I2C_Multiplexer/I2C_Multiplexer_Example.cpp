// TCA9548A 1-to-8 I2C multiplexer
// https://www.adafruit.com/products/2717?gclid=CK31zdzswMwCFZSMaQodqhMDxw

// DESCRIPTION

// You just found the perfect I2C sensor, and you want to wire up two or three or more of them to your Arduino when you realize "Uh oh, this chip has a fixed I2C address, and from what I know about I2C, you cannot have two devices with the same address on the same SDA/SCL pins!" Are you out of luck? You would be, if you didn't have this ultra-cool TCA9548A 1-to-8 I2C multiplexer!

// Finally, a way to get up to 8 same-address I2C devices hooked up to one microcontroller - this multiplexer acts as a gatekeeper, shuttling the commands to the selected set of I2C pins with your command.

// Using it is fairly straight-forward: the multiplexer itself is on I2C address 0x70 (but can be adjusted from 0x70 to 0x77) and you simply write a single byte with the desired multiplexed output number to that port, and bam - any future I2C packets will get sent to that port. In theory, you could have 8 of these multiplexers on each of 0x70-0x77 addresses in order to control 64 of the same-I2C-addressed-part.

// Like all Adafruit breakouts, we put this nice chip on a breakout for you so you can use it on a breadboard with capacitors, and pullups and pulldowns to make usage a snap. Some header is required and once soldered in you can plug it into a solderless-breadboard. The chip itself is 3V and 5V compliant so you can use it with any logic level.

// Multiplexer can extend I2C bus in 8 branch directions.
// Could be used for level shifting or selecting various branches of an I2C bus

// One time use example - set the multiplexer channel to 0 just using for level shifting
enum MultiplexerAddress {MULTIPLEXER_ADDRESS_DEFAULT=0x70}; // default I2C bus address
enum MultiplexerChannels {ch0=0, ch1, ch2, ch3, ch4, ch5, ch6, ch7};
I2C* I2CMultiplexer;
I2CMultiplexer = new I2C(I2C::kMXP, MULTIPLEXER_ADDRESS_DEFAULT);  // Using Expansion Chassis I2C port; Multiplexer address on 0x70
if (I2CMultiplexer->WriteBulk(0x00, 1))printf ( "\n\n\nWriteBulk operation failed! line %d\n\n\n", __LINE__ ); // write 1 byte address of the channel (0) to switch to on the multiplexer
delete I2CMultiplexer;  // not going to change the multiplexer port after this so done with object
I2CMultiplexer = NULL;
