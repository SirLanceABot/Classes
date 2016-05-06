#include "JoyStickCompensation.h"
#include <iostream>
#include <stdio.h>

int main() // test JoyStickCompensation
{
    for (double x=-1.06f; x<=1.06f; x+=.01f)
	{
	printf("%f, ", x);

	for (int i=1; i<=3; i++) printf("%f, ", JoyStickCompensation(x, i));

	printf("\n");
	}

    return 0;
}
