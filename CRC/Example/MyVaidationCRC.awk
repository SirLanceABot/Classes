# make sure at least the Start of line character field 1 and a CRC field 2
  NF <= 1 {
	print "No CRC", FNR, $0 >> "badcrc.txt";
	next;}

# check for Start of line character
  $1 != "#," {
	print "No Start Character ", FNR, $0 >> "badcrc.txt";
	next;}

# check the CRC-16
  $2 != sprintf("%0004x,",CRC16(substr($0, 10))) {
	print "bad CRC", FNR, $0 >> "badcrc.txt";
	next;}

# everything else is good; could check for other validation of fields such as values are existence
	{print substr($0, 10) >> "good.csv";}
	
# todo - try to find an embedded start character to save the end.  Unlikely to help much, though
#($1 != "#,") {if(index($0, "#") != 0){$0 = substr($0, index($0, "#"));} else next; }
#index(substr($0, 2), "#") != 0 {$0 = substr($0, index(substr($0, 2), "#")+length("#"));}

#sample data
#  #, da00, 46.178016, 1183625c6e7, 46.217748, 1184643451, 2327, 509093, 1, 2.329005, 451
#  #, 1214, 46.237612, 1185152378, 2328, 508927, 1, 2.334145, 450
#  #, 0830, 48.462563, 1242169183, 2440, 509049, 1, 2.329005, 450
#  #, 
#  78a2, 48.383100, 1240132637, 2436, 509051, 1, 2.329005, 450

#Example usage:
#gawk -f crc.awk -f MyValidationCRC.awk InputFileToValidate

#C:\Documents and Settings\Richard Thomas\My Documents\Rick\PC & Electronics\My-PC-Updates\unix_type_utilities\gawk-good>
#gawk
# -f "C:\Documents and Settings\Richard Thomas\My Documents\Robotics\C-Pgm\EclipseRIO\GitHub\CRC\crc.awk"
# -f "C:\Documents and Settings\Richard Thomas\My Documents\Robotics\C-Pgm\EclipseRIO\GitHub\CRC\Example\crc.awk"
# "C:\Documents and Settings\Richard Thomas\My Documents\Robotics\C-Pgm\EclipseRIO\GitHub\CRC\Example\crc.awk"