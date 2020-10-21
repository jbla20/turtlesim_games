#include <iostream>
#include "stdlib.h"
#include "time.h"

void printArray(int array[10])
{
    for (int i = 0; i < 10; i++)
    {
        std::cout << "Num: " << array[i] << "\n";
    }
    std::cout << "\n";
}

int main(int argc, char *argv[])
{

    //Creating an array of random numbers
    int randNum[10];

    for (int i = 0; i < 10; i++)
    {
        int randomNumber = rand() % 10 + 1;
        randNum[i] = randomNumber;
    }

    //Printing the array
    printArray(randNum);

    //Sorting the array

    return 0;
}