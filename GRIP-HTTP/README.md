Example of usage of JACKSON json parser.
JACKSON is distributed with FRC WPILib as of 2020.

Example of Java program reading GRIP lines report transmitted via HTTP.
HTTP can transmit a consistent set of data in one GET but be careful that the GET is a blocking function
and performance is not measured and could be slow.
Note that the port number in GRIP is incorrectly documented and this Java program uses the correct number
as used in GRIP (for example, http://127.0.0.1:2084/GRIP/data Note that other directories are also used - see the
GRIP documentation in GitHub)
