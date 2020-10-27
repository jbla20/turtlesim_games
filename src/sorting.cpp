#include <iostream>
#include <vector>
#include "stdlib.h"
#include "time.h"

void quicksort(int *iList, int firstIndex, int lastIndex)
{
    int pivot, i, j, tmp;

    if (firstIndex < lastIndex)
    {
        pivot = firstIndex;
        i = firstIndex;
        j = lastIndex;

        while (i < j)
        {
            while (iList[i] <= iList[pivot] && i < lastIndex)
            {
                i++;
            }
            while (iList[j] > iList[pivot])
            {
                j--;
            }
            if (i < j)
            {
                tmp = iList[i];
                iList[i] = iList[j];
                iList[j] = tmp;
            }
        }
        tmp = iList[pivot];
        iList[pivot] = iList[j];
        iList[j] = tmp;

        quicksort(iList, firstIndex, j - 1);
        quicksort(iList, j + 1, lastIndex);
    }
}

void printArray(int *array, int size)
{
    for (int i = 0; i < size; i++)
    {
        std::cout << "Num: " << array[i] << "\n";
    }
    std::cout << "\n";
}

int main(int argc, char *argv[])
{

    //Creating an array of random numbers
    int size = 10;
    int randNum[size];
    //std::vector<int> sortedArray;

    srand(time(NULL));

    for (int i = 0; i < 10; i++)
    {
        int randomNumber = rand() % 10 + 1;
        randNum[i] = randomNumber;
    }

    //Printing the original array
    std::cout << "Original array: \n";
    printArray(randNum, size);

    //Sorting the array
    quicksort(randNum, 0, size - 1);

    //Printing the sorted array
    std::cout << "Sorted array: \n";
    printArray(randNum, size);

    return 0;
}