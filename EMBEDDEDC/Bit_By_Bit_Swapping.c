/*Bit-by-Bit Reversal
When you reverse the bits of a 32-bit integer bit by bit, you swap the first bit with the 32nd bit, the second bit with the 31st bit, and so on. 
This operation completely reverses the order of all bits in the integer.

For example:

Original (binary): 11011110101011011011111011101111 (0xDEADBEEF)
Reversed (binary): 11110111101110111101011010111101 (0xF77DB57B)

*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
char* DecToBin(int num)
{
    char* bin = (char*)malloc(33); // Allocate memory for 32 bits + null terminator
    if (bin == NULL) {
        fprintf(stderr, "Memory allocation failed in DecToBin function\n");
        return NULL; // Check if memory allocation was successful
    }
    bin[32] = '\0'; // Set the null terminator at the end of the string
    int idx;
    for (idx = 31; idx >= 0; idx--)
    {
        if (num & (1 << idx)) // Check if the bit is set
            bin[31 - idx] = '1'; // Set the corresponding index in the string to '1'
        else
            bin[31 - idx] = '0'; // Set the corresponding index in the string to '0'
    }
    return bin;
}


// Function to reverse bits of a 32-bit integer
uint32_t reverseBits(uint32_t n) {
    uint32_t reversed = 0;
    for (int i = 0; i < 32; i++) {
        // Extract the least significant bit of n and add it to the reversed number
        reversed |= (n & 1) << (31 - i);
        // Shift n to the right to process the next bit
        n >>= 1;
    }
    return reversed;
}

int main() {
    uint32_t x = 0xDEADBEEF;
    // Convert number to binary string
    char* binStr = DecToBin(x);
    if (binStr == NULL) {
        fprintf(stderr, "Failed to convert number to binary string in main function\n");
        return 1;
    }

  printf("Original vaue in binary: %s\n", binStr);
  printf("Original value in hex: 0x%X\n", x);
    uint32_t reversed = reverseBits(x);
    
    // Convert swapped number to binary string
    char* RevBinStr = DecToBin(reversed);
    if (RevBinStr == NULL) {
        fprintf(stderr, "Failed to convert swapped number to binary string in main function\n");
        free(binStr);
        binStr = NULL;
        return 1;
    }
    printf("Reverse value by bit by bit swapping technique in binary: %s\n", RevBinStr);
    printf("Reversed value in hex: 0x%X\n", reversed);

    
    // Free allocated memory
    free(binStr);
    binStr = NULL;
    free(RevBinStr);
    RevBinStr = NULL;
    return 0;
}