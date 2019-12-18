package frc.robot;

import java.util.Date;

class Id {
	
// prints the class and date, for example:	
static String __FILE__(Class<?> c)
	{
		try {
			String classFileName = c.getSimpleName() + ".class"; // simple name like MyClass.class
			Date d = new Date(c.getResource(classFileName).openConnection().getLastModified()); // date
			classFileName = classFileName + " was compiled on " + d;
			return classFileName;
		} catch (Exception e) {System.out.println(e);}
		
		return "Id threw an error\n";
	}
//String className = c.getResource(classFileName).getPath(); // complete name with path (optional)-probably not useful on roboRIO

// prints the line number
static String __LINE__()
{

	final StackTraceElement[] ste = Thread.currentThread().getStackTrace();
	String returnLine="";
	for (int i=0; i<ste.length; i++)
		if (ste[i].getMethodName().equals("__LINE__")) {
			returnLine = returnLine + ste[i+1].getFileName() +'@'+ ste[i+1].getLineNumber();
			break;
		}
	return returnLine;
}

}
