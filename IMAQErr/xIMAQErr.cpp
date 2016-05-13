#include "IMAQErr.h"

#include <iostream>

void main()
{
    std::cout << IMAQErrorMessage(0) << std::endl;
    std::cout << IMAQErrorMessage(0xBFF69000) << std::endl;
    std::cout << IMAQErrorMessage(0xBFF6900A) << std::endl;
    std::cout << IMAQErrorMessage(99) << std::endl;

}