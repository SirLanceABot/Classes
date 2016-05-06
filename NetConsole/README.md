FIRST FRC NetConsole replacement program

NetConsole from FIRST is very limited in its performance.  Large quantities of data cannot be accurately gathered.  As the window size increases, performance slows and UDP packets from the roboRIO are missed.

If a lot of data needs to be collected at high speed, then this is the program for you.  Run in place of NetConsole and your data are written to disk at high speed.

If the data need to be viewed as they are being collected then use in tandem and log viewing program such as logview.

Dynamic Log Viewer
version 1.7.1
www.mx-3.cz
http://tringi.trimcore.cz/Dynamic_Log_Viewer

For some reason this program cannot run at the same time as the "real" NetConsole (This program will not start if NetConsole is running).  It can however run with Eclipse "NetConsole" plugin RLog running.

This program was compiled on the Windows DriverStation with Eclipse MinGW(64) GCC C++ compiler.  Previous versions compiled with Microsoft Visual Studio Express, Cygwin, and MinGW (32).  Some pertinent statements for those other IDEs are included as comments.
