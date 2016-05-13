#include "IMAQErr.h"

#include <iostream>
#include <stdio.h>

int main()
{
    char message[250];

    IMAQErrorMessage(0, message);          std::cout << message << std::endl;

    for (int i = (signed int)0xBFF69000; i< (signed int)0xBFF69000 + 78; i++)
	{IMAQErrorMessage(i, message);
	printf("%8X %d %s\n", i, i, message);}

    return 0;
}
