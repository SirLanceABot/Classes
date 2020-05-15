
LSD
======================

A Java port of the excellent Line Segment Detector algorithm from:

http://www.ipol.im/pub/art/2012/gjmr-lsd/


Compilation
======================

Simply run : javac *.java

Then java GUI

ToDo
======================
* Add methods for accepting java images in LSD.java
* Tidy up my comments in LSD.java
* Instead of using error() throw std. exceptions when necessary

How this version was made
======================
RKT copied the GitHub repository LSD from user anfractuosity

Add make some minor changes:
    * add public to several field variables (instead of making Getters)
    * add color and dashes to the line segment display - better to see the generated lines
    
